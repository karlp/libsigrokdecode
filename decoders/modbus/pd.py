## Modbus is a protocol where you have a client and one or more servers. This
## decoder is for Modbus RTU.
## The RX channel will be checked for both client->server and server->client
## communication, the TX channel only for client->server.

import sigrokdecode as srd

RX = 0
TX = 1


class Data:
    def __init__(self, start, end, data):
        self.start = start
        self.end = end
        self.data = data


class Modbus_ADU:
    """ An Application Data Unit is what MODBUS calls one message """
    def __init__(self, parent, start, write_channel):
        """Start new message, starting at start"""
        self.data = []
        self.parent = parent
        self.start = start
        self.last_read = start
        self.write_channel = write_channel

    def add_data(self, start, end, data):
        ptype, rxtx, pdata = data
        self.last_read = end
        if ptype == 'DATA':
            self.data.append(Data(start, end, pdata[0]))

    def writeClientServerMessages(self, channel, annotation_prefix):
        data = self.data
        put = self.parent.puta

        if len(data) < 4:
            if len(data) == 0:
                # Sometimes happens with noise
                return
            put(data[0].start, data[-1].end,
                annotation_prefix + "data",
                "Message to short to be legal Modbus")
            return

        if len(data) > 256:
            put(data[0].start, data[-1].end,
                annotation_prefix + "data",
                "Modbus data frames are limited to 256 bytes")
            return

        server_id = data[0].data
        message = ""
        if server_id == 0:
            message = "Broadcast message"
        elif 1 <= server_id <= 247:
            message = "Slave ID: {}".format(server_id)
        elif 248 <= server_id <= 255:
            message = "Slave ID: {} (reserved address)".format(server_id)
        put(data[0].start, data[0].end,
            annotation_prefix + "server-id",
            message)

        function = data[1].data
        if function >= 1 and function <= 4:
            self.write_read_data_command(annotation_prefix)
        else:
            put(data[1].start, data[-3].end,
                annotation_prefix + "data",
                "Unknown function: {}".format(data[1].data))

        # Check CRC
        crc_byte1, crc_byte2 = self.calc_crc()
        if data[-2].data == crc_byte1 and data[-1].data == crc_byte2:
            put(data[-2].start, data[-1].end, annotation_prefix + 'crc',
                "CRC correct")
        else:
            put(data[-2].start, data[-1].end, annotation_prefix + 'crc',
                "CRC should be {} {}".format(crc_byte1, crc_byte2))

    def write_message(self):
        if self.write_channel == RX:
            self.writeClientServerMessages(RX, 'rx-Cs-')
        if self.write_channel == TX:
            self.writeClientServerMessages(RX, 'tx-')

    def write_read_data_command(self, annotation_prefix):
        """ Interpret a command to read x units of data starting at address, ie
        functions 1,2,3 and 4, and write the result to the annotations """
        data = self.data
        put = self.parent.puta
        function = data[1].data
        functionname = {1: "Read Coils",
                        2: "Read Discrete Inputs",
                        3: "Read Holding Registers",
                        4: "Read Input Registers",
                        }[function]

        put(data[1].start, data[1].end, annotation_prefix + "function",
            "Function {}: {}".format(function, functionname))

        if len(data) < 8:
            # All of these functions need a slave ID, a function, two bytes of
            # address, two bytes of quantity, and two bytes of CRC
            put(data[2].start, data[-1].end, annotation_prefix + "data",
                "Message too short to be legal {}".format(functionname))
            return

        starting_address = self.half_word(2)
        # Some instruction manuals use a long form name for addresses, this is
        # listed here for convienience.
        # Example: holding register 60 becomes 30061.
        address_name = 10000 * function + 1 + starting_address
        put(data[2].start, data[3].end,
            annotation_prefix + "starting-address",
            "Start at address {:X} / {:d}".format(starting_address,
                                                  address_name))

        put(data[4].start, data[5].end,
            annotation_prefix + 'data',
            "Read {:d} units of data".format(self.half_word(4)))

    def half_word(self, start):
        """ Return the half word (16 bit) value starting at start bytes in. If
        it goes out of range it raises the usual errors. """
        return self.data[start].data * 0x100 + self.data[start+1].data

    def calc_crc(self, end=-2):
        """ Calculate the CRC, as described in the spec """
        result = 0xFFFF
        magic_number = 0xA001  # as defined in the modbus specification
        for byte in self.data[:end]:
            result = result ^ byte.data
            for i in range(8):
                LSB = result & 1
                result = result >> 1
                if (LSB):  # if the LSB is true
                    result = result ^ magic_number
        byte1 = result & 0xFF
        byte2 = (result & 0xFF00) >> 8
        return (byte1, byte2)


class Decoder(srd.Decoder):
    api_version = 2
    id = 'modbus'
    name = 'Modbus'
    longname = 'Modbus RTU over RS232/RS485'
    desc = 'Modbus RTU protocol for industrial applications'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = ['modbus']
    annotations = (
        ('rx-Sc-server-id', ''),
        ('rx-Sc-function', ''),
        ('rx-Sc-crc', ''),
        ('rx-Sc-starting-address', ''),
        ('rx-Sc-data', ''),
        ('rx-Cs-server-id', ''),
        ('rx-Cs-function', ''),
        ('rx-Cs-crc', ''),
        ('rx-Cs-starting-address', ''),
        ('rx-Cs-data', ''),
        ('tx-server-id', ''),
        ('tx-function', ''),
        ('tx-crc', ''),
        ('tx-starting-address', ''),
        ('tx-data', ''),
    )
    annotation_rows = (
        ('rx-cs', 'Rx data, client->server', (0, 1, 2, 3, 4)),
        ('rx-sc', 'Rx data, server->client', (5, 6, 7, 8, 9)),
        ('tx', 'Tx data, client->server', (10, 11, 12, 13, 14)),
    )

    def __init__(self, **kwargs):
        self.ADURx = None
        self.ADUTx = None

        self.bitlength = None

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        if rxtx == TX:
            self.decode_correct_ADU(ss, es, data, self.ADUTx)
        if rxtx == RX:
            self.decode_correct_ADU(ss, es, data, self.ADURx)

    def puta(self, start, end, annotation_channel_text, message):
        """ Put an annotation from start to end, with annotation_channel as a
        string """
        annotation_channel = [s[0] for s in self.annotations].index(annotation_channel_text)
        self.put(start, end, self.out_ann,
                 [annotation_channel, [message]])

    def decode_correct_ADU(self, ss, es, data, ADU):
        ptype, rxtx, pdata = data

        # We need to know how long bits are before we can start decoding
        # messages
        if self.bitlength is None:
            if ptype == "STARTBIT" or ptype == "STOPBIT":
                self.bitlength = es - ss
            else:
                return

        if ADU is None:
            self.start_new_decode(ss, es, data)
            return

        # According to the modbus spec, there should be 3.5 characters worth of
        # space between each message, and a character is 11 bits long
        # Round down for reliablility
        if 0 <= (ss - ADU.last_read) <= self.bitlength * 38:
            ADU.add_data(ss, es, data)
        else:
            ADU.write_message()
            self.start_new_decode(ss, es, data)

    def start_new_decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        if rxtx == TX:
            self.ADUTx = Modbus_ADU(self, ss, TX)
        if rxtx == RX:
            self.ADURx = Modbus_ADU(self, ss, RX)

        self.decode(ss, es, data)

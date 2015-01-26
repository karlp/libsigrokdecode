## Modbus is a protocol where you have a client and one or more servers. This
## decoder is for Modbus RTU.
## The RX channel will be checked for both client->server and server->client
## communication, the TX channel only for client->server.

import sigrokdecode as srd

RX = 0
TX = 1


class No_more_data(Exception):
    """ This exception is a signal that we should stop parsing an ADU as there
    is no more data to parse """
    pass


class Data:
    def __init__(self, start, end, data):
        self.start = start
        self.end = end
        self.data = data


class Modbus_ADU_CS:
    """ An Application Data Unit is what MODBUS calls one message.
    Protocol decoders are supposed to keep track of state and then provide
    decoded data to the backend as it reads it. In modbus's case, the state is
    the ADU up to that point. This class represents the state and writes the
    messages to the backend.
    CS stands for Client -> Server """
    def __init__(self, parent, start, write_channel, annotation_prefix):
        self.data = []
        self.parent = parent
        self.start = start
        self.last_read = start
        self.write_channel = write_channel
        self.last_byte_put = -1
        self.annotation_prefix = annotation_prefix
        # Any modbus message needs to be at least 4 long. The modbus function
        # may make this longer
        self.minimum_length = 4

    def add_data(self, start, end, data):
        ptype, rxtx, pdata = data
        self.last_read = end
        if ptype == 'DATA':
            self.data.append(Data(start, end, pdata[0]))
            self.parse()

    def put_if_needed(self, byte_to_put, annotation, message):
        """ This class keeps track of how much of the data has already been
        annotated. This function tells the parent class to write message, but
        only if it hasn't written about this bit before.
        byte_to_put: only write if it hasn't yet written byte_to_put. It will
           write from the start of self.last_byte_put+1 to the end of
           byte_to_put.
        annotation: annotation to write to, without annotation_prefix
        message: message to write"""
        if byte_to_put > len(self.data)-1:
            # if the byte_to_put hasn't been read yet
            raise No_more_data

        if byte_to_put > self.last_byte_put:
            self.parent.puta(
                self.data[self.last_byte_put+1].start,
                self.data[byte_to_put].end,
                self.annotation_prefix + annotation,
                message)
            self.last_byte_put = byte_to_put
            raise No_more_data

    def close(self, message_overflow):
        """ Function to be called when next message is started. As there is
        always space between one message and the next, we can use that space
        for errors at the end. """
        # TODO: figure out how to make this happen for last message
        data = self.data
        if len(data) < self.minimum_length:
            if len(data) == 0:
                # Sometimes happens with noise, safe to ignore
                return
            self.parent.puta(
                data[self.last_byte_put].start, message_overflow,
                self.annotation_prefix + "data",
                "Message too short or not finished")
        if len(data) > 256:
            try:
                self.put_if_needed(
                    len(data)-1,
                    self.annotation_prefix + "data",
                    "Modbus data frames are limited to 256 bytes")
            except No_more_data:
                pass

    def parse(self):
        data = self.data
        try:
            server_id = data[0].data
            message = ""
            if server_id == 0:
                message = "Broadcast message"
            elif 1 <= server_id <= 247:
                message = "Slave ID: {}".format(server_id)
            elif 248 <= server_id <= 255:
                message = "Slave ID: {} (reserved address)".format(server_id)
            self.put_if_needed(0, "server-id", message)

            function = data[1].data
            if function >= 1 and function <= 4:
                self.parse_read_data_command()
            else:
                self.put_if_needed(1, "data",
                                   "Unknown function: {}".format(data[1].data))
                self.put_if_needed(len(data)-1, "data", "Unknown function")

            # if the message gets here without raising an exception, the
            # message goes on longer than it should

            self.put_if_needed(len(data)-1, "data", "Message too long")

        except No_more_data:
            # this is just a message saying we don't need to parse anymore this
            # round
            pass

    def check_CRC(self, byte_to_put):
        """ Check the CRC code, data[byte_to_put] is the second byte of the CRC
        """
        # Check CRC
        crc_byte1, crc_byte2 = self.calc_crc(byte_to_put)
        data = self.data
        if data[-2].data == crc_byte1 and data[-1].data == crc_byte2:
            self.put_if_needed(byte_to_put, 'crc', "CRC correct")
        else:
            self.put_if_needed(
                byte_to_put, 'crc',
                "CRC should be {} {}".format(crc_byte1, crc_byte2))

    def parse_read_data_command(self):
        """ Interpret a command to read x units of data starting at address, ie
        functions 1,2,3 and 4, and write the result to the annotations """
        data = self.data
        function = data[1].data
        functionname = {1: "Read Coils",
                        2: "Read Discrete Inputs",
                        3: "Read Holding Registers",
                        4: "Read Input Registers",
                        }[function]

        self.put_if_needed(1, "function",
                           "Function {}: {}".format(function, functionname))

        self.minimum_length = max(self.minimum_length, 8)

        starting_address = self.half_word(2)
        # Some instruction manuals use a long form name for addresses, this is
        # listed here for convienience.
        # Example: holding register 60 becomes 30061.
        address_name = 10000 * function + 1 + starting_address
        self.put_if_needed(
            3, "starting-address",
            "Start at address 0x{:X} / {:d}".format(starting_address,
                                                    address_name))

        self.put_if_needed(5, 'data',
                           "Read {:d} units of data".format(self.half_word(4)))
        self.check_CRC(7)

    def half_word(self, start):
        """ Return the half word (16 bit) value starting at start bytes in. If
        it goes out of range it raises the usual errors. """
        if start+1 > len(self.data)-1:
            # If there isn't enough length to access data[start+1]
            raise No_more_data
        return self.data[start].data * 0x100 + self.data[start+1].data

    def calc_crc(self, last_byte):
        """ Calculate the CRC, as described in the spec
        The last byte of the crc should be data[last_byte] """
        if last_byte < 3:
            # every modbus ADU should be as least 4 long, so we should never
            # have to calculate a CRC on something shorter
            raise Exception("Could not calculate CRC: message too short")

        result = 0xFFFF
        magic_number = 0xA001  # as defined in the modbus specification
        for byte in self.data[:last_byte-1]:
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
        annotation_channel = \
            [s[0] for s in self.annotations].index(annotation_channel_text)
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
            # if there is any data in the ADU
            if len(ADU.data) > 0:
                # extend errors for 3 bits after last byte, we can guarentee
                # space
                ADU.close(ADU.data[-1].end + self.bitlength * 3)
            self.start_new_decode(ss, es, data)

    def start_new_decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        if rxtx == TX:
            self.ADUTx = Modbus_ADU_CS(self, ss, TX, "tx-")
        if rxtx == RX:
            self.ADURx = Modbus_ADU_CS(self, ss, RX, "rx-Cs-")

        self.decode(ss, es, data)

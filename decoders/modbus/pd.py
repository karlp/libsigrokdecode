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

        message= ""

        if len(data) < 4:
            if len(data) == 0:
                # Sometimes happens with noise
                return
            put(data[0].start, data[-1].end,
                annotation_prefix + "data",
                "Message to short to be legal Modbus")
            return

        server_id = data[0].data
        if server_id == 0:
            message = "Broadcast message"
        elif 1 <= server_id <= 247:
            message = "Slave ID: {}".format(server_id)
        elif 248 <= server_id <=  255:
            message = "Slave ID: {} (reserved address)".format(server_id)
        put(data[0].start, data[0].end,
            annotation_prefix + "server-id",
            message)

        function = data[1].data
        if function == 3:
            put(data[1].start, data[1].end,
                annotation_prefix + "function",
                "Function 3: read holding registers")
            if len(data) < 8:
                put(data[2].start, data[-1].end,
                    annotation_prefix + "data",
                    "Message too short to be legal Read Holding Register")
                return

            starting_register = self.half_word(2)
            put(data[2].start, data[3].end,
                annotation_prefix + "starting-address",
                "Start at address {:X} / {:d}".format(starting_register,
                                                    starting_register + 40001))
            put(data[4].start, data[5].end,
                annotation_prefix + 'data',
                "Read {:d} registers".format(self.half_word(4)))
        else:
            put(data[1].start, data[-3].end,
                annotation_prefix + "data",
                "Unknown function: {}".format(data[1].data))

    def write_message(self):
        if self.write_channel == RX:
            self.writeClientServerMessages(RX, 'rx-Cs-')
        if self.write_channel == TX:
            self.writeClientServerMessages(RX, 'tx-')

    def half_word(self, start):
        """ Return the half word (16 bit) value starting at start bytes in. If
        it goes out of range it raises the usual errors. """
        return self.data[start].data * 0x100 + self.data[start+1].data


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
        ('rx-cs', 'Rx data, assuming all data is client->server', (0,1,2,3,4)),
        ('rx-sc', 'Rx data, assuming all data is server->client', (5,6,7,8,9)),
        ('tx', 'Tx data, assuming all data is client->server', (10,11,12,13,14)),
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
        """ put an annotation from start to end, with annotation_channel as a string """
        annotation_channel = [s[0] for s in self.annotations].index(annotation_channel_text)
        self.put(start, end, self.out_ann,
                        [annotation_channel, [message]])

    def decode_correct_ADU(self, ss, es, data, ADU):
        ptype, rxtx, pdata = data

        # We need to know how long bits are before we can start decoding messages
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

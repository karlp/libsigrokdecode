import sigrokdecode as srd


RX = 0
TX = 1

class Modbus_ADU:
    def __init__(self, parent, start, write_channel):
        """Start new message, starting at start"""
        self.data = []
        self.parent = parent
        self.start = start
        self.last_read = start
        self.write_channel = write_channel
        self.error = False

    def add_data(self, data, message_end):
        ptype, rxtx, pdata = data
        self.last_read = message_end
        if ptype == 'DATA':
            self.data.append((pdata[0], message_end))

    def message(self):
        message = []
        data = [d[0]  for d in self.data]
        self.error = False

        if len(data) < 4:
            self.error = True
            return "Message to short to be legal Modbus"

        address = data[0]
        if address == 0:
            message.append("Broadcast message")
        elif 1 <= address <= 247:
            message.append("Slave ID: {}".format(address))
        elif 248 <= address <=  255:
            message.append("Slave ID: {} (reserved address)".format(address))
            self.error = True

        function = data[1]
        if function == 3:
            if len(data) < 8:
                error = True
                return "Message too short to be legal Read Holding Register"
            starting_register = data[2] * 0x100 + data[3]
            number_of_registers = data[4] * 0x100 + data[5]
            message.append("Read {} holding registers starting at {}".format(
                number_of_registers, 
                starting_register))
        else: 
            message.append("Unknown function")

        
        return "; ".join(message)

    def write_message(self):
        message = self.message()

        if self.error:
            self.write_channel += 2

        self.parent.put(self.start, 
                        self.last_read, 
                        self.parent.out_ann, 
                        [self.write_channel, [message]])



class Decoder(srd.Decoder):
    api_version = 2
    id = 'modbus'
    name = 'Modbus'
    longname = 'Modbus RTU over RS232'
    desc = 'Modbus RTU protocol for industrial applications'
    license = 'gplv2+'
    inputs = ['uart']
    outputs = ['modbus']
    annotations = (
        ('rx_description', 'What is happening on the line configured as Rx'),
        ('tx_description', 'What is happening on the line configured as Tx'),
        ('rx_error', 'What is happening on the line configured as Rx'),
        ('tx_error', 'What is happening on the line configured as Tx'),
    )
    annotation_rows = (
        ('rx', 'Rx data', (0,2)),
        ('tx', 'Tx data', (1,3)),
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

        # According to the modbus spek, there should be 3.5 characters worth of
        # space between each message, and a character is 10 bits long
        if 0 <= (ss - ADU.last_read) <= self.bitlength * 35:
            ADU.add_data(data, es)
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

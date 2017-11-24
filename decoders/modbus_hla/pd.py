##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2017 Karl Palsson <karlp@etactica.com>
##
## Licensed under your choice of BSD 2 clause, X11, ISC, Apache 2.0 or ISC licenses
##

import sigrokdecode as srd
from math import ceil

class Decoder(srd.Decoder):
    api_version = 3
    id = 'modbus_hla'
    name = 'Modbus HLA'
    longname = 'Modbus higher level analysis'
    desc = 'Analysing protocol layer modbus.'
    license = 'bsd2'
    inputs = ['modbus']
    outputs = ['modbus_hla']
    annotations = (
        ('sc-frame', ''),
        ('cs-frame', ''),
        ('broadcasts', ''),
        ('errors', ''),
    )
    annotation_rows = (
        ('fr', "Entire Frames", (0, 1, 2,)),
        ('errors', 'Errors', (3,)),
    )
    options = (
    )

    def __init__(self):
        self.waiting = None
        self.waiting_s = None

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        print("decode called for ", ss, es, data)
        if self.waiting:
            print("was waiting for ", self.waiting)
        else:
            print("ws NOT waiting for anything")

        ptype, obj = data
        sid = obj["server_id"]
        function = obj["function"]
        if ptype == "Sc":
            self.put(ss, es, self.out_ann, [0, ["Slave: {}({:#x}), function: {}".format(sid, sid, function)]])
            if self.waiting:
                if self.waiting != sid:
                    self.put(self.waiting_s, es, self.out_ann, [2, ["Reply from wrong device! {} != {}".format(self.waiting, sid)]])
                self.waiting = None
                self.waiting_s = None
            else:
                self.put(ss, es, self.out_ann, [3, ["Unsolicited modbus reply!"]])
                
        if ptype == "Cs":
            if sid == 0:
                self.put(ss, es, self.out_ann, [1, ["BROADCAST, function: {}".format(function)]])
            else:
                self.put(ss, es, self.out_ann, [2, ["Slave: {}({:#x}), function: {}".format(sid, sid, function)]])
            if self.waiting:
                self.put(self.waiting_s, es, self.out_ann, [3, ["Missing reply from {}({:#x})".format(self.waiting, self.waiting)]])
            # no expected response to broadcasts
            if sid > 0:
                self.waiting = sid
                self.waiting_s = es
            else:
                self.waiting = None
                self.waiting_s = None

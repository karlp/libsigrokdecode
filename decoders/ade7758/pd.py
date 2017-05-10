##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2017 Karl Palsson <karlp@etactica.com>
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.

import math
import sigrokdecode as srd
from .lists import *


class Decoder(srd.Decoder):
    api_version = 2
    id = 'ade7758'
    name = 'ADE7758'
    longname = 'Analog Devices ADE7758 polyphase power metering IC'
    desc = 'Decodes spi register read/writes'
    license = 'mit'
    inputs = ['spi']
    outputs = ['ade7758']
    annotations = (
        ('read', 'Register read commands'),
        ('write', 'Register write commands'),
        ('warning', 'Warnings'),
    )
    annotation_rows = (
        ('read', 'Read', (0,)),
        ('write', 'Write', (1,)),
        ('warnings', 'Warnings', (2,)),
    )

    def reset(self):
        self.expected = 0
        self.mosi_bytes = []
        self.miso_bytes = []

    def __init__(self):
        self.ss_cmd, self.es_cmd = 0, 0
        self.reset()

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putx(self, data):
        self.put(self.ss_cmd, self.es_cmd, self.out_ann, data)

    def put_warn(self, pos, msg):
        self.put(pos[0], pos[1], self.out_ann, [2, [msg]])

    def decode(self, ss, es, data):
        ptype = data[0]
        if ptype == 'CS-CHANGE':
            # Bear in mind, that CS is optional according to the datasheet
            # If we transition high mid-stream, toss out our data and restart.
            cs_old, cs_new = data[1:]
            #print("cs-change: old, new: , expected, current", cs_old, cs_new, self.expected, len(self.mosi_bytes))
            if cs_old is not None and cs_old == 0 and cs_new == 1:
                if len(self.mosi_bytes) > 0 and len(self.mosi_bytes[1:]) < self.expected:
                    # mark short read/write for reg at least!
                    #print("short read (CS deasserted early): %d < %d" % (len(self.mosi_bytes[1:]), self.expected))
                    self.es_cmd = es
                    write = self.cmd & 0x80
                    reg = self.cmd & 0x7f
                    rblob = regs.get(reg)
                    if write:
                        self.putx([1, ['%s: %s' % (rblob[0], "SHORT")]])
                    else:
                        self.putx([0, ['%s: %s' % (rblob[0], "SHORT")]])

                    self.put_warn([self.ss_cmd, es], "Short transfer!")
                self.reset()
            return

        # Don't care about anything else.
        if ptype != 'DATA':
            return
        mosi, miso = data[1:]

        if len(self.mosi_bytes) == 0:
            self.ss_cmd = ss
        self.mosi_bytes.append(mosi)
        self.miso_bytes.append(miso)
        #print("entered decode with len (mosi) now: ", len(self.mosi_bytes))

        # A transfer is 2-4 bytes, (command+1..3 byte reg)
        if len(self.mosi_bytes) < 2:
            return

        self.cmd = self.mosi_bytes[0]
        write = self.cmd & 0x80
        reg = self.cmd & 0x7f
        rblob = regs.get(reg)
        if not rblob:
            # if you don't have CS, this will _destroy_ comms!
            self.put_warn([self.ss_cmd, es], 'Unknown register!')
            return

        self.expected = math.ceil(rblob[3] / 8)
        if len(self.mosi_bytes[1:]) != self.expected:
            return
        valo = None
        vali = None
        self.es_cmd = es
        if self.expected == 3:
            valo = self.mosi_bytes[1] << 16 | self.mosi_bytes[2] << 8 | self.mosi_bytes[3]
            vali = self.miso_bytes[1] << 16 | self.miso_bytes[2] << 8 | self.miso_bytes[3]
        elif self.expected == 2:
            valo = self.mosi_bytes[1] << 8 | self.mosi_bytes[2]
            vali = self.miso_bytes[1] << 8 | self.miso_bytes[2]
        elif self.expected == 1:
            valo = self.mosi_bytes[1]
            vali = self.miso_bytes[1]
        else:
            # raise? programming error to get here.
            print("A programming error has occurred :(")

        # Hard hex for now.  PV needs support for this!
        if write:
            self.putx([1, ['%s: %#x' % (rblob[0], valo)]])
        else:
            self.putx([0, ['%s: %#x' % (rblob[0], vali)]])
        self.reset()

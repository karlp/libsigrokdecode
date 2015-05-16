##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

import sigrokdecode as srd

regs = {
	0: "Product_ID",
	1: "Revision_ID",
	2: "Motion",
	3: "Delta_X",
	4: "Delta_Y",
	5: "SQUAL",
	6: "Shutter_Upper",
	7: "Shutter_Lower",
	8: "Maximum_Pixel",
	9: "Pixel_Sum",
	0xa: "Minimum_Pixel",
	0xb: "Pixel_Grab",
	0xd: "Mouse_Control",
	0x3a: "Chip_Reset",
	0x3f: "Inv_Rev_ID",
	0x63: "Motion_Burst"
}

class Decoder(srd.Decoder):
    api_version = 2
    id = 'adns5020_spi'
    name = 'ADNS 5020 (SPI)'
    longname = 'ADNS 5020 Optical Mouse sensor (SPI)'
    desc = 'Bidirectonal command and data over SPI like protocol.'
    license = 'gplv2'
    inputs = ['spi']
    outputs = ['ands5020_spi']
    annotations = (
        ('adns5020', 'ADNS Command/Data'),
    )

    def __init__(self, **kwargs):
        self.ss_cmd, self.es_cmd = 0, 0
        self.mosi_bytes = []

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putx(self, data):
        self.put(self.ss_cmd, self.es_cmd, self.out_ann, data)

    def decode(self, ss, es, data):
        ptype, mosi, miso = data

        # Only care about data packets.
        if ptype != 'DATA':
            return
        self.ss, self.es = ss, es

        # TODO - look at CS? how to resync if it starts capture mid trace?
        if len(self.mosi_bytes) == 0:
            self.ss_cmd = ss
        self.mosi_bytes.append(mosi)

        # writes/reads are always two transfers
        # TODO - lies! burst mode is different
        if len(self.mosi_bytes) != 2:
            return

        self.es_cmd = es
        cmd, data = self.mosi_bytes
        write = cmd & 0x80
        reg = cmd & 0x7f
        reg_desc = regs.get(cmd, "Reserved %#x" % cmd)
        if cmd > 0x63:
            reg_desc = "Unknown"
        if write:
            self.putx([0, ["%s=>%#x" % reg_desc, data]])
        else:
            self.putx([0, ["%s<=%d" % (reg_desc, data)]])

        self.mosi_bytes = []

#
# This file is part of the libsigrokdecode project.
#
# Copyright (C) 2013 Uwe Hermann <uwe@hermann-uwe.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
#

import sigrokdecode as srd


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
        ('text-verbose', 'Human-readable text (verbose)'),
    )

    def decode_read_registers(self, data=1234321):
        if data == 1234321:
            print("no data supplied. Self: {}".format(self))
        if self.byte == 2:
            self.cd_function_data = {}
        if self.byte == 3:
            self.cd_function_data['address'] = data*256
        if self.byte == 4:
            self.cd_function_data['address'] += data
        if self.byte == 5:
            self.cd_function_data['bytes to read'] = data*256
        if self.byte == 6:
            self.cd_function_data['bytes to read'] += data
        if self.byte == 7:
            self.cd_function_data['checksum'] = data*256
        if self.byte == 8:
            self.cd_function_data['checksum'] += data
            self.end_ADU("Read {} holding registers starting from {}".format(
                self.cd_function_data['bytes to read'],
                self.cd_function_data['address']))

    def __init__(self, **kwargs):
        self.decoder_chooser = {
            3: self.decode_read_registers,
            }

        self.set_empty_state()

    def set_empty_state(self):
        self.state = 'IDLE'
        self.es = None
        self.start_of_ADU = None
        self.server_id = None
        self.cd_function = None  # Current Decoding function
        self.cd_function_data = None
        self.byte = None

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        # We don't care about stop and start bits
        if ptype != 'DATA':
            return

        data_as_byte = pdata[0]

        self.es = es

        if self.state == 'IDLE':
            self.server_id = data_as_byte
            self.start_of_ADU = ss
            self.state = 'WAITING FOR FUNCTION CODE'
            self.byte = 1  # server id is 0, function code is 1
        elif self.state == 'WAITING FOR FUNCTION CODE':
            self.byte += 1
            self.cd_function = self.decoder_chooser.get(data_as_byte, None)

            if self.cd_function is not None:
                self.state = 'READING'
                self.cd_function(data_as_byte)
            else:
                self.end_ADU("Bad format")
        elif self.state == 'READING':
            self.byte += 1
            self.cd_function(data_as_byte)

    def end_ADU(self, message):
        self.put(self.start_of_ADU, self.es, self.out_ann, [0, [message]])
        self.set_empty_state()

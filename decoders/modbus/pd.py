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


RX = 0
TX = 1
Modbus_wait_max = 10


class Modbus_ADU:
    def __init__(self, start):
        """Start new message, starting at start"""
        self.data = []
        self.start = start
        self.last_read = start

    def message(self):
        return "Starts at {}, ends at {}".format(self.start, self.last_read)

    def add_data(self, data, message_end):
        ptype, rxtx, pdata = data
        self.last_read = message_end
        if ptype == 'DATA':
            self.data.append(pdata[0])


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

    def __init__(self, **kwargs):
        self.set_empty_state()

    def set_empty_state(self):
        self.state = 'IDLE'
        self.ADU = None

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def decode(self, ss, es, data):
        ptype, rxtx, pdata = data

        # for now, only look at channel TX
        if rxtx == TX:
            return

        if self.state == 'IDLE':
            self.state = 'READING'
            self.ADU = Modbus_ADU(ss)

        # At this point the state should be READING
        if 0 <= (ss - self.ADU.last_read) <= Modbus_wait_max:
            self.ADU.add_data(data, es)
        else:
            self.end_ADU(self.ADU)
            self.decode(ss, es, data)

    def end_ADU(self, ADU):
        self.put(ADU.start, ADU.last_read, self.out_ann, [0, [ADU.message()]])
        self.set_empty_state()

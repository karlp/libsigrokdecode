##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2014 Johannes Roemer <jroemer@physik.uni-wuerzburg.de>
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

'''
This decoder handles the proprietary single wire communication protocol used
by the Aosong AM230x/DHTxx series of digital humidity and temperature sensors.

Sample rate:
A sample rate of at least 200kHz is recommended to properly detect all the
elements of the protocol.

Options:
The AM230x and DHTxx digital humidity and temperature sensors use the same
single-wire protocol with different encoding of the measured values.
Therefore the option 'device' must be used to properly decode the
communication of the respective sensor.
'''

from .pd import Decoder

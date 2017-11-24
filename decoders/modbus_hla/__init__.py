##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2017 Karl Palsson <karlp@etactica.com>
##
## Your choice of BSD 2 clause, ISC, Apache 2.0, MIT, or X11 Licenses

'''
This decoder stacks on top of the 'modbus' PD and does some frame level analysis
for things like missing replies, and includes simpler, higher level annotations
'''

from .pd import Decoder

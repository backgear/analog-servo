##
## This file is part of the libsigrokdecode project.
##
## Decodes analog-servo's Monitor.c binary telemetry frame -- either channel
## 1 of the composite-USB build's two CDC-ACM ports, or (L431) the dedicated
## Monitor UART (USART1/PA9) -- see docs/monitor-protocol.md and Monitor.c.
## Stacks on sigrok's built-in 'uart' decoder: point it at a capture of
## that byte stream (e.g. a UART bridge, or USB bulk data fed in as a raw
## logic/async-serial-shaped capture) with the uart PD underneath this one
## in PulseView's decoder stack.
##
## DRAFT -- not run against a real capture yet. See pd.py's top-of-file
## comment for the one part (the uart PD's exact Python-output tuple
## shape) that's worth double-checking against your installed sigrok
## version before trusting this.
##

'''
This decoder stacks on the 'uart' PD and decodes analog-servo's Monitor.c
20-byte binary telemetry frame (see docs/monitor-protocol.md for the full
frame layout):

  offset  size  field
  0       1     sync (0xAA)
  1       4     seq (uint32 LE)
  5       4     positionCounts (int32 LE)
  9       4     velocityCounts (int32 LE)
  13      4     torqueCommand (int32 LE)
  17      1     status (bits0-1 mode, bit2 enabled, bit3 fault latched,
                 bit4 reserved (was idle reduced, Idle.c -- deleted, always 0
                 now), bit5 hw enable asserted, bit6 synthetic,
                 bit7 cycle_start -- see docs/monitor-protocol.md)
  18      1     faultReason
  19      1     crc8 (poly 0x07, init 0, MSB-first, over bytes 0..18)

Config-tier values (gains, etc.) are deliberately not in this frame -- it's
real-time state only; query config via the text CLI instead. See
docs/monitor-protocol.md's "Frame" section.
'''

from .pd import Decoder

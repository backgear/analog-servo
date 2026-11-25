"""Binary monitor-telemetry frame decode: sync byte, CRC-8, resyncing.

Shared by anything that reads the raw binary stream (`Monitor.c`'s wire format,
see `docs/monitor-protocol.md`) directly, rather than through `monitor_viewer.py`'s
own live connection. Split out of what used to be a combined SCPI-bridge tool so
that reading this format doesn't pull in SCPI-instrument-loopback code that has
nothing to do with it.
"""
import struct
import sys
from pathlib import Path

# tools/monitor_frame.py is the canonical, schema-derived frame layout (see
# its own top comment). This file used to hand-maintain an independent copy
# of the struct format and byte offsets, which is exactly what went stale
# and silently broke this decoder the last time the frame grew -- the sync
# byte, status-bit and fault-reason checks below were all reading the wrong
# bytes, and it decoded zero frames instead of erroring loudly. Importing
# instead of duplicating means this file can't drift out of sync with
# Monitor.c's real wire format again. sys.path is adjusted explicitly
# rather than relying on the caller's cwd, since export_run.py (same
# directory) is normally run as a standalone script, which only puts THIS
# file's own directory on sys.path, not tools/.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from monitor_frame import STRUCT_FMT, FRAME_LEN, FIELD_OFFSETS, crc8

SIGNALS = ('error', 'torque', 'velocity', 'position', 'reference', 'reference_velocity')
SCALES = dict(error=1., torque=256., velocity=1., position=256., reference=256., reference_velocity=1.)

_STATUS_OFFSET = FIELD_OFFSETS['status']
_FAULT_REASON_OFFSET = FIELD_OFFSETS['fault_reason']


class Frames:
    """Resyncing decoder: feed() raw bytes, get back (seq, fields) for each valid frame."""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while self.buffer:
            if self.buffer[0] != 0xaa:
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_LEN:
                break
            frame = self.buffer[:FRAME_LEN]
            if (crc8(frame[:-1]) != frame[-1] or frame[_FAULT_REASON_OFFSET] > 5
                    or frame[_STATUS_OFFSET] & 3 == 3):
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_LEN]
            (_, seq, pos, vel, torque, ref, rv, integral, elec_angle, hall_sector,
             comm_offset, status, fault, _) = struct.unpack(STRUCT_FMT, frame)
            # Error validity cannot be inferred for all controller branches from this stream.
            yield seq, dict(position=pos, velocity=vel, torque=torque,
                            reference=ref, reference_velocity=rv, error=ref - pos,
                            velocity_integral_q16=integral,
                            electrical_angle_deg=elec_angle, hall_sector=hall_sector,
                            commutation_offset_deg=comm_offset,
                            logic=((status >> 2) & 1) | (bool(fault or status & 8) << 1)
                            | (((status >> 5) & 7) << 2) | (1 << 5))

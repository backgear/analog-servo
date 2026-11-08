"""Binary monitor-telemetry frame decode: sync byte, CRC-8, 28-byte frames.

Shared by anything that reads the raw binary stream (`Monitor.c`'s wire format,
see `docs/monitor-protocol.md`) directly, rather than through `monitor_viewer.py`'s
own live connection. Split out of what used to be a combined SCPI-bridge tool so
that reading this format doesn't pull in SCPI-instrument-loopback code that has
nothing to do with it.
"""
import struct

SIGNALS = ('error', 'torque', 'velocity', 'position', 'reference', 'reference_velocity')
SCALES = dict(error=1., torque=256., velocity=1., position=256., reference=256., reference_velocity=1.)


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
    return crc


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
            if len(self.buffer) < 28:
                break
            frame = self.buffer[:28]
            if crc8(frame[:-1]) != frame[-1] or frame[26] > 5 or frame[25] & 3 == 3:
                del self.buffer[0]
                continue
            del self.buffer[:28]
            _, seq, pos, vel, torque, ref, rv, status, fault, _ = struct.unpack('<BIiiiiiBBB', frame)
            # Error validity cannot be inferred for all controller branches from this stream.
            yield seq, dict(position=pos, velocity=vel, torque=torque,
                            reference=ref, reference_velocity=rv, error=ref - pos,
                            logic=((status >> 2) & 1) | (bool(fault or status & 8) << 1)
                            | (((status >> 5) & 7) << 2) | (1 << 5))

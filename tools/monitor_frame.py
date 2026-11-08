"""Shared decoder for Monitor.c's binary telemetry frame -- see
../docs/monitor-protocol.md for the wire format. Deliberately stdlib-only
(just `struct`): monitor_viewer.py's own GUI needs pygame, but anything
that just wants to decode a stream of frames (a headless capture script,
an offline converter) shouldn't have to pull that in too.
"""
import struct

FRAME_LEN = 28   # was 20 before commandedPosition/commandedVelocity (2x int32) were added
SYNC_BYTE = 0xAA
STRUCT_FMT = "<BIiiiiiBBB"

MODE_NAMES = ("stepdir", "velocity", "analog")
FAULT_REASON_NAMES = ("none", "polarity", "zero_encoder", "endstop", "following_error", "sustained_current")


def crc8(data):
    """Same table-less CRC-8 as Monitor.c's Crc8() and the sigrok decoder's
    crc8() -- poly 0x07, init 0, MSB-first, no reflect/xor-out."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def decode_frame(data):
    """data is exactly FRAME_LEN bytes starting with the sync byte. Returns
    a dict on a valid frame (CRC-8 checks out and fault_reason is in
    range), None on a false lock."""
    (sync, seq, position, velocity, torque, cmd_position, cmd_velocity,
     status, fault_reason, frame_crc) = struct.unpack(STRUCT_FMT, data)

    if crc8(data[:-1]) != frame_crc:
        return None
    if sync != SYNC_BYTE or (status & 3) >= len(MODE_NAMES) or fault_reason >= len(FAULT_REASON_NAMES):
        return None

    return {
        "seq": seq,
        "position": position,
        "velocity": velocity,
        "torque": torque,
        "cmd_position": cmd_position,
        "cmd_velocity": cmd_velocity,
        "mode": MODE_NAMES[status & 0x3],
        "enabled": bool((status >> 2) & 1),
        "fault_latched": bool((status >> 3) & 1),
        "idle_reduced": bool((status >> 4) & 1),
        "hw_enable": bool((status >> 5) & 1),
        "synthetic": bool((status >> 6) & 1),
        "cycle_start": bool((status >> 7) & 1),
        "fault_reason": FAULT_REASON_NAMES[fault_reason],
    }


class FrameSync:
    """Sliding-window resync, same approach as the sigrok decoder
    (tools/sigrok-decoder-monitor): find a candidate sync byte, try to
    decode+CRC-check a full frame there, and on a false lock drop just
    that one byte and keep looking rather than discarding a whole
    FRAME_LEN-byte window and risking losing a real sync byte inside it."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        frames = []
        self.buf.extend(data)
        while True:
            idx = self.buf.find(SYNC_BYTE)
            if idx < 0:
                self.buf.clear()
                break
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < FRAME_LEN:
                break
            frame = decode_frame(bytes(self.buf[:FRAME_LEN]))
            if frame is not None:
                frames.append(frame)
                del self.buf[:FRAME_LEN]
            else:
                del self.buf[:1]
        return frames

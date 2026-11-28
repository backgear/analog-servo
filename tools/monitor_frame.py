"""Shared decoder for Monitor.c's binary telemetry frame -- see
../docs/monitor-protocol.md for the wire format. Deliberately stdlib-only
(just `struct`): monitor_viewer.py's own GUI needs pygame, but anything
that just wants to decode a stream of frames (a headless capture script,
an offline converter) shouldn't have to pull that in too.
"""
import struct

SYNC_BYTE = 0xAA

# Declarative mirror of Monitor.c's AppendSchema() calls (what
# MonitorGetSchemaJson() serves as `monitor schema`): field name and wire
# type, in wire order. STRUCT_FMT/FRAME_LEN/FIELD_OFFSETS below are all
# DERIVED from this list by struct_format()/field_offsets(), not
# hand-typed -- a field added or removed here is the only change needed on
# the Python side. This is exactly the discipline Monitor.h's own doc
# comment on MonitorGetSchemaJson() asks host tools to follow (derive from
# the schema, don't cache a hand-copied struct) -- see
# tools/sigrok/monitor_frames.py and tools/test_sigrok_monitor.py, which
# both import FIELD_OFFSETS instead of hardcoding byte positions, for the
# bug that hand-copying caused when this frame last grew (commutation_offset_deg
# was the 4th field added since FRAME_LEN was 20: commandedPosition/
# commandedVelocity, then velocityIntegral on `main` independently, then
# electricalAngleDeg, then hallSector, then this one).
FIELDS = (
    ("sync", "u8"), ("seq", "u32"), ("position", "i32"), ("velocity", "i32"),
    ("torque_command", "i32"), ("commanded_position", "i32"), ("commanded_velocity", "i32"),
    ("velocity_integral", "i32"), ("electrical_angle_deg", "i32"), ("hall_sector", "i32"),
    ("commutation_offset_deg", "i32"), ("status", "u8"), ("fault_reason", "u8"), ("crc8", "u8"),
)

_WIRE_TYPE_CHAR = {"u8": "B", "u32": "I", "i32": "i"}


def struct_format(fields):
    """(name, type) pairs in wire order -> a little-endian struct.unpack
    format string. Takes the same shape MonitorGetSchemaJson()'s "fields"
    list uses (name/type; offset is derived here instead of read, so a
    live-fetched schema works the same way as the FIELDS literal above).
    Assumes no padding between fields, true of any __attribute__((packed))
    struct like MonitorFrame."""
    return "<" + "".join(_WIRE_TYPE_CHAR[kind] for _, kind in fields)


def field_offsets(fields):
    """(name, type) pairs -> {name: byte offset}, walked in the same order
    struct_format() encodes -- so this always agrees with STRUCT_FMT's
    actual layout, including after FIELDS gains or loses a field."""
    offsets, pos = {}, 0
    for name, kind in fields:
        offsets[name] = pos
        pos += struct.calcsize("<" + _WIRE_TYPE_CHAR[kind])
    return offsets


STRUCT_FMT = struct_format(FIELDS)
FRAME_LEN = struct.calcsize(STRUCT_FMT)
FIELD_OFFSETS = field_offsets(FIELDS)

MODE_NAMES = ("stepdir", "host", "analog")
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
    (sync, seq, position, velocity, torque, cmd_position, cmd_velocity, velocity_integral,
     electrical_angle_deg, hall_sector, commutation_offset_deg, status, fault_reason,
     frame_crc) = struct.unpack(STRUCT_FMT, data)

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
        # Bit4 multiplexes this wire field during additive excitation.
        "cmd_velocity": 0 if status & 0x10 else cmd_velocity,
        "excitation": bool(status & 0x10),
        "injected_torque": cmd_velocity if status & 0x10 else 0,
        # Divide by 65536.0 for torque-command units.
        "velocity_integral_q16": velocity_integral,
        # Commutation.c's electrical angle (degrees, 0..359),
        # output.topology=two_phase_analog only -- see docs/two-phase-drive.md.
        # Stale at 0 on any build/run that never drives that topology.
        "electrical_angle_deg": electrical_angle_deg,
        # RAW Hall sector (0..5, or -1 for an invalid/wiring-fault state --
        # see Hall.h), NOT the derived electrical_angle_deg above --
        # output.topology=two_phase_analog only; stale at 0 otherwise.
        "hall_sector": hall_sector,
        # ConfigStore's commutation.offset_deg -- the LEARNED alignment
        # offset `commutation learn` persists (see docs/two-phase-drive.md).
        # 0 until a successful learn attempt; not gated to two-phase-active
        # like the two fields above, since it's a plain config read.
        "commutation_offset_deg": commutation_offset_deg,
        "mode": MODE_NAMES[status & 0x3],
        "enabled": bool((status >> 2) & 1),
        "fault_latched": bool((status >> 3) & 1),
        "idle_reduced": False,  # reserved legacy display key
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

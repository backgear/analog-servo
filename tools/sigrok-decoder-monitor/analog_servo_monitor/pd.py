##
## This file is part of the libsigrokdecode project.
##
## analog-servo Monitor.c binary telemetry decoder. See __init__.py.
##
## UNVERIFIED AGAINST A REAL CAPTURE -- drafted from Monitor.c's source and
## docs/monitor-protocol.md's frame layout, not bench-tested against sigrok.
## The one piece most likely to need fixing against your installed sigrok
## version: how the stacked 'uart' PD emits each received byte over its
## Python output. This decoder assumes each call to self.wait() returns
## (ss, es, data) with data a tuple whose second element is 'DATA' and whose
## third element is (value, ...) -- matching sigrok's decoders/uart/pd.py as
## of the 2023-ish releases, via `self.register(srd.OUTPUT_PYTHON)` on the
## uart PD and the convention that RX bytes carry rxtx index 0. If your
## sigrok build emits a different shape, this is the one block to adjust
## (see decode() below) -- everything after byte extraction (frame assembly,
## struct.unpack_from, CRC check, annotations) doesn't depend on that detail
## and should need no changes.
##

import struct
import sigrokdecode as srd

ANN_SYNC, ANN_SEQ, ANN_POSITION, ANN_VELOCITY, ANN_TORQUE, \
    ANN_CMD_POSITION, ANN_CMD_VELOCITY, \
    ANN_STATUS, ANN_FAULT_REASON, ANN_CRC, ANN_FRAME, ANN_WARNING = range(12)

FRAME_LEN = 28   # was 20 before commandedPosition/commandedVelocity (2x int32) were added
SYNC_BYTE = 0xAA

MODE_NAMES = ('stepdir', 'velocity', 'analog')  # status bits0-1, 2 bits
FAULT_REASON_NAMES = ('none', 'polarity', 'zero_encoder', 'endstop', 'following_error')
# Config-tier values (gains, etc.) are deliberately NOT in the frame -- it's
# real-time state only, see docs/monitor-protocol.md's "Frame" section. Query
# config via the text CLI instead.


def crc8(data):
    '''Same table-less CRC-8 as Monitor.c's Crc8() -- poly 0x07, init 0,
    MSB-first, no reflect/xor-out.'''
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class Decoder(srd.Decoder):
    api_version = 3
    id = 'analog_servo_monitor'
    name = 'analog-servo Monitor'
    longname = 'analog-servo Monitor.c binary telemetry'
    desc = "Decodes analog-servo's 28-byte Monitor.c telemetry frame."
    license = 'gplv2+'
    inputs = ['uart']
    outputs = []
    tags = ['Util']
    annotations = (
        ('sync', 'Sync byte'),
        ('seq', 'Sequence number'),
        ('position', 'Position (counts, measured)'),
        ('velocity', 'Velocity (counts/tick, measured)'),
        ('torque', 'Torque command'),
        ('cmd-position', 'Commanded position (counts, desired)'),
        ('cmd-velocity', 'Commanded velocity (counts/tick, desired)'),
        ('status', 'Status bits'),
        ('fault-reason', 'Fault reason'),
        ('crc', 'CRC-8'),
        ('frame', 'Frame'),
        ('warning', 'Warning'),
    )
    annotation_rows = (
        ('fields', 'Fields', (ANN_SYNC, ANN_SEQ, ANN_POSITION, ANN_VELOCITY,
                               ANN_TORQUE, ANN_CMD_POSITION, ANN_CMD_VELOCITY,
                               ANN_STATUS, ANN_FAULT_REASON, ANN_CRC)),
        ('frames', 'Frames', (ANN_FRAME,)),
        ('warnings', 'Warnings', (ANN_WARNING,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.window = []     # sliding list of (ss, es, value), max FRAME_LEN long

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putf(self, ss, es, ann_idx, text):
        self.put(ss, es, self.out_ann, [ann_idx, [text]])

    def try_emit_frame(self, window):
        '''Returns True if window is a real frame (CRC-8 checks out, and
        emits annotations for it), False if it's a false lock on a
        coincidental sync byte (and emits nothing). The CRC-8 (same
        poly/init as Monitor.c's Crc8(), see crc8() above) is real error
        detection, not just plausibility -- a false lock on a payload byte
        that happens to equal 0xAA will fail the CRC check essentially
        always (1/256 chance of a coincidental match), unlike the old
        mode/fault_reason-range-only check this replaced.'''
        data = bytes(b[2] for b in window)
        (sync, seq, position, velocity, torque, cmd_position, cmd_velocity,
         status, fault_reason, frame_crc) = struct.unpack_from('<BIiiiiiBBB', data)

        if crc8(data[:-1]) != frame_crc:
            return False

        mode_bits = status & 0x3
        if fault_reason >= len(FAULT_REASON_NAMES):
            return False

        self.putf(window[0][0], window[0][1], ANN_SYNC, 'sync=0x%02X' % sync)
        self.putf(window[1][0], window[4][1], ANN_SEQ, 'seq=%u' % seq)
        self.putf(window[5][0], window[8][1], ANN_POSITION, 'position=%d' % position)
        self.putf(window[9][0], window[12][1], ANN_VELOCITY, 'velocity=%d' % velocity)
        self.putf(window[13][0], window[16][1], ANN_TORQUE, 'torque=%d' % torque)
        self.putf(window[17][0], window[20][1], ANN_CMD_POSITION, 'cmd_position=%d' % cmd_position)
        self.putf(window[21][0], window[24][1], ANN_CMD_VELOCITY, 'cmd_velocity=%d' % cmd_velocity)

        mode = MODE_NAMES[mode_bits]
        enabled = (status >> 2) & 1
        fault_latched = (status >> 3) & 1
        reserved4 = (status >> 4) & 1   # was idle_reduced, Idle.c deleted -- always 0, see __init__.py
        hw_enable = (status >> 5) & 1
        synthetic = (status >> 6) & 1
        cycle_start = (status >> 7) & 1
        self.putf(window[25][0], window[25][1], ANN_STATUS,
                  'mode=%s enabled=%d fault=%d reserved4=%d hw_enable=%d synthetic=%d cycle_start=%d' %
                  (mode, enabled, fault_latched, reserved4, hw_enable, synthetic, cycle_start))

        reason = FAULT_REASON_NAMES[fault_reason]
        self.putf(window[26][0], window[26][1], ANN_FAULT_REASON, 'fault_reason=%s' % reason)

        self.putf(window[27][0], window[27][1], ANN_CRC, 'crc8=0x%02X OK' % frame_crc)

        self.putf(window[0][0], window[-1][1], ANN_FRAME,
                  'seq=%u pos=%d(cmd %d) vel=%d(cmd %d) torque=%d mode=%s%s%s%s' %
                  (seq, position, cmd_position, velocity, cmd_velocity, torque, mode,
                   ' FAULT:%s' % reason if fault_latched else '',
                   ' SYNTHETIC' if synthetic else '',
                   ' CYCLE_START' if cycle_start else ''))
        return True

    def decode(self):
        while True:
            # See this file's top comment: adjust this block first if your
            # sigrok's 'uart' PD emits a different Python-output shape.
            ss, es, data = self.wait()
            if not (isinstance(data, tuple) and len(data) >= 2 and data[0] == 'DATA'):
                continue
            rxtx, pdata = data[1], data[2]
            if rxtx != 0:   # 0 == RX -- Monitor.c's data only ever flows device->host
                continue
            value = pdata[0] if isinstance(pdata, tuple) else pdata

            self.window.append((ss, es, value))
            if len(self.window) > FRAME_LEN:
                self.window.pop(0)

            # Sliding window: checks EVERY byte position as a candidate
            # frame start, not just "the next one after a failed lock" --
            # a false lock just means this particular window gets rejected
            # and we keep sliding, one byte at a time, rather than
            # discarding FRAME_LEN bytes and losing whatever real sync byte
            # might be among them.
            if len(self.window) == FRAME_LEN and self.window[0][2] == SYNC_BYTE:
                if self.try_emit_frame(self.window):
                    self.window = []   # consumed -- the next FRAME_LEN bytes should
                                          # be the next real frame, starting fresh
                    continue
                # else: false lock (bad CRC or implausible field), rejected --
                # fall through and keep sliding

"""Check telemetry decoding without serial hardware or GUI dependencies."""
import struct
import unittest

from monitor_frame import FRAME_LEN, STRUCT_FMT, SYNC_BYTE, FrameSync, crc8, decode_frame


def frame(seq=123, status=0x25, reason=0, sync=SYNC_BYTE):
    data = struct.pack(STRUCT_FMT, sync, seq, -42, 7, -1000, 99, -8,
                       -123 * 65536, 270, 4, -12, status, reason, 0)
    return data[:-1] + bytes([crc8(data[:-1])])


class MonitorFrameTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc8(b'123456789'), 0xF4)

    def test_fields_and_signed_values(self):
        data = frame()
        self.assertEqual(len(data), FRAME_LEN)
        decoded = decode_frame(data)
        self.assertEqual(decoded['seq'], 123)
        self.assertEqual(decoded['position'], -42)
        self.assertEqual(decoded['torque'], -1000)
        self.assertEqual(decoded['cmd_position'], 99)
        self.assertEqual(decoded['cmd_velocity'], -8)
        self.assertEqual(decoded['velocity_integral_q16'], -123 * 65536)
        self.assertEqual(decoded['electrical_angle_deg'], 270)
        self.assertEqual(decoded['hall_sector'], 4)
        self.assertEqual(decoded['commutation_offset_deg'], -12)
        self.assertEqual(decoded['mode'], 'host')
        self.assertTrue(decoded['enabled'])
        self.assertTrue(decoded['hw_enable'])
        self.assertFalse(decoded['fault_latched'])

    def test_reject_invalid_frames(self):
        corrupt = bytearray(frame())
        corrupt[6] ^= 1
        for data in (corrupt, frame(status=3), frame(reason=255), frame(sync=0)):
            with self.subTest(data=data):
                self.assertIsNone(decode_frame(data))

    def test_partial_frames_and_resync(self):
        stream = b'noise' + bytes([SYNC_BYTE]) + b'bad' + frame(1) + frame(2)
        decoder = FrameSync()
        decoded = []
        for byte in stream:
            decoded.extend(decoder.feed(bytes([byte])))
        self.assertEqual([f['seq'] for f in decoded], [1, 2])
        self.assertEqual(decoder.feed(b''), [])


if __name__ == '__main__':
    unittest.main()

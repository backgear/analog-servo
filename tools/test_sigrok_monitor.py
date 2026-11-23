"""Check both sigrok readers against the same current telemetry frame.

The protocol-decoder test stubs sigrok's annotation API, not a hardware capture.
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from test_monitor_frame import frame

ROOT = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SigrokMonitorTests(unittest.TestCase):
    def test_stream_reader_resync_and_integral(self):
        reader = load('monitor_frames', ROOT / 'sigrok/monitor_frames.py').Frames()
        decoded = list(reader.feed(b'noise' + frame()))
        self.assertEqual(len(decoded), 1)
        seq, fields = decoded[0]
        self.assertEqual(seq, 123)
        self.assertEqual(fields['position'], -42)
        self.assertEqual(fields['velocity_integral_q16'], -123 * 65536)

    def test_protocol_decoder_offsets_and_invalid_mode(self):
        stub = types.SimpleNamespace(Decoder=object)
        with patch.dict(sys.modules, sigrokdecode=stub):
            pd = load('monitor_pd', ROOT / 'sigrok-decoder-monitor/analog_servo_monitor/pd.py')
        decoder = pd.Decoder()
        annotations = []
        decoder.putf = lambda *args: annotations.append(args)
        window = [(i, i + 1, value) for i, value in enumerate(frame())]
        self.assertTrue(decoder.try_emit_frame(window))
        self.assertIn((25, 29, pd.ANN_INTEGRAL, 'velocity_integral_q16=-8060928'), annotations)
        self.assertTrue(any(a[:3] == (29, 30, pd.ANN_STATUS) for a in annotations))
        self.assertFalse(decoder.try_emit_frame(
            [(i, i + 1, value) for i, value in enumerate(frame(status=3))]))

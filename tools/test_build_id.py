"""Check build-ID extraction and board comparison without serial hardware."""
import contextlib
import io
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

import build_id

GNU_ID = bytes.fromhex('0f72b558fe7ce22e0007b40e8f45501af969a4dc')


def note(name, n_type, desc):
    pad = lambda b: b + b'\0' * (-len(b) % 4)
    return struct.pack('<3I', len(name), len(desc), n_type) + pad(name) + pad(desc)


def elf32(notes):
    """Minimal ELF32 LE: header, one SHT_NOTE section, null section header first."""
    body = b''.join(notes)
    shoff = 52 + len(body)
    header = bytearray(52)
    header[:6] = b'\x7fELF\x01\x01'
    struct.pack_into('<I', header, 0x20, shoff)
    struct.pack_into('<HH', header, 0x2E, 40, 2)
    null_sh = bytes(40)
    note_sh = struct.pack('<10I', 0, build_id.SHT_NOTE, 2, 0x08000000, 52, len(body), 0, 0, 4, 0)
    return bytes(header) + body + null_sh + note_sh


class ElfBuildIdTests(unittest.TestCase):
    def test_finds_gnu_note_after_other_notes(self):
        data = elf32([note(b'ARM\0', 1, b'\x01\x02\x03'),
                      note(b'GNU\0', build_id.NT_GNU_BUILD_ID, GNU_ID)])
        self.assertEqual(build_id.elf_build_id(data), GNU_ID.hex())

    def test_none_without_build_id_note(self):
        self.assertIsNone(build_id.elf_build_id(elf32([note(b'ARM\0', 1, b'\x01')])))

    def test_rejects_non_elf32(self):
        with self.assertRaises(ValueError):
            build_id.elf_build_id(b'\x7fELF\x02\x01' + bytes(58))

    def test_real_firmware_elf_if_built(self):
        elves = sorted(build_id.FIRMWARE_DIR.glob('.pio/build/*/firmware.elf'))
        if not elves:
            self.skipTest('no firmware build')
        for elf in elves:
            self.assertRegex(build_id.elf_build_id(elf.read_bytes()), r'^[0-9a-f]{40}$', str(elf))

    def test_real_firmware_bin_carries_the_note(self):
        """The flashed image, not just the ELF, holds the note the board reads."""
        elves = sorted(build_id.FIRMWARE_DIR.glob('.pio/build/*/firmware.elf'))
        if not elves:
            self.skipTest('no firmware build')
        for elf in elves:
            desc = bytes.fromhex(build_id.elf_build_id(elf.read_bytes()))
            whole = note(b'GNU\0', build_id.NT_GNU_BUILD_ID, desc)
            image = (elf.parent / 'firmware.bin').read_bytes()
            # The .bin starts at flash base; a note placed in RAM would also
            # land in it, but ~384 MiB in. 512 KiB is the largest flash here.
            self.assertLess(len(image), 512 * 1024, str(elf))
            self.assertIn(whole, image, str(elf))


class CompareTests(unittest.TestCase):
    def run_main(self, board):
        with tempfile.TemporaryDirectory() as tmp:
            elf = Path(tmp) / 'firmware.elf'
            elf.write_bytes(elf32([note(b'GNU\0', build_id.NT_GNU_BUILD_ID, GNU_ID)]))
            out = io.StringIO()
            with mock.patch.object(build_id, 'board_build_id', return_value=board), \
                    contextlib.redirect_stdout(out):
                status = build_id.main(['/dev/null', '--elf', str(elf)])
        return status, out.getvalue()

    def test_match_exits_zero(self):
        status, out = self.run_main(GNU_ID.hex())
        self.assertEqual(status, 0)
        self.assertIn('MATCH', out)

    def test_mismatch_exits_one(self):
        status, out = self.run_main('00' * 20)
        self.assertEqual(status, 1)
        self.assertIn('matches none', out)

    def test_reply_value_ignores_echo_and_other_lines(self):
        reply = 'get build.id\r\nbuild.id=abc123\r\n\r\n'
        self.assertEqual(build_id.reply_value(reply, 'build.id'), 'abc123')
        self.assertIsNone(build_id.reply_value('error=unknown variable\r\n', 'build.id'))


if __name__ == '__main__':
    unittest.main()

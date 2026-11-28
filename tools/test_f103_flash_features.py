"""Check that FLASH_IS_SCARCE changes linked features, not just build flags.

Run after building analog_servo_f103 and analog_servo_f103_128k.
"""
from pathlib import Path
import struct
import unittest

BUILD_DIR = Path(__file__).resolve().parents[1] / 'firmware/.pio/build'


def symbols(data):
    """Read defined symbols from the little-endian ELF32 firmware."""
    shoff = struct.unpack_from('<I', data, 0x20)[0]
    shentsize, shnum = struct.unpack_from('<HH', data, 0x2E)
    sections = [struct.unpack_from('<10I', data, shoff + i * shentsize)
                for i in range(shnum)]
    result = {}
    for section in sections:
        if section[1] != 2:  # SHT_SYMTAB
            continue
        strings = sections[section[6]]
        names = data[strings[4]:strings[4] + strings[5]]
        for offset in range(section[4], section[4] + section[5], section[9]):
            name, value, _, info, _, index = struct.unpack_from('<IIIBBH', data, offset)
            if index:
                result[names[name:].split(b'\0', 1)[0].decode()] = (value, info >> 4)
    return result


class FlashFeatureTests(unittest.TestCase):
    def test_capacity_selects_tuning_and_uart(self):
        for env, scarce in (('analog_servo_f103', True),
                            ('analog_servo_f103_128k', False)):
            with self.subTest(env=env):
                path = BUILD_DIR / env / 'firmware.elf'
                if not path.exists():
                    self.skipTest(f'{env} not built')
                linked = symbols(path.read_bytes())
                for name in ('AutotuneStartRelay', 'RlsTick', 'GainGlideStart', 'UsartInit'):
                    self.assertEqual(name in linked, not scarce, name)
                # The startup file retains a weak default vector when UART is absent.
                self.assertEqual(linked['USART1_IRQHandler'][1], 2 if scarce else 1)
                self.assertEqual(linked['__config_flash_start'][0],
                                 0x0800FC00 if scarce else 0x0801FC00)
                self.assertLess(linked['__build_id_start'][0],
                                linked['__config_flash_start'][0])


if __name__ == '__main__':
    unittest.main()

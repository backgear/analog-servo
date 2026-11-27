#!/usr/bin/env python3
"""Say which local build, if any, is the image running on the board.

python3 tools/build_id.py PORT
python3 tools/build_id.py --socket /tmp/servo.sock
python3 tools/build_id.py PORT --elf firmware/.pio/build/analog_servo_f103/firmware.elf

Reads `get build.id` from the board and compares it with the GNU build-ID
note in each ELF (default: every firmware/.pio/build/*/firmware.elf). If
serial_console.py already owns PORT, the query goes through its socket,
since the console opens the port exclusively.

The ID hashes the whole linked ELF, debug info included, so a rebuild that
changes only debug info gives a new ID for identical flash bytes. That
errs toward "mismatch", never toward a false "match".

Exit status: 0 match, 1 no match, 2 the board or an ELF could not be read.
"""
import argparse
from datetime import datetime
from pathlib import Path
import struct
import sys
import time

FIRMWARE_DIR = Path(__file__).resolve().parent.parent / 'firmware'
SHT_NOTE = 7
NT_GNU_BUILD_ID = 3


def elf_build_id(data):
    """Hex GNU build-ID of a little-endian ELF32 image, or None if it has none."""
    if data[:4] != b'\x7fELF' or data[4] != 1 or data[5] != 1:
        raise ValueError('not a little-endian ELF32 file')
    shoff, = struct.unpack_from('<I', data, 0x20)
    shentsize, shnum = struct.unpack_from('<HH', data, 0x2E)
    for index in range(shnum):
        _, sh_type, _, _, offset, size = struct.unpack_from('<6I', data, shoff + index * shentsize)
        if sh_type != SHT_NOTE:
            continue
        pos, end = offset, offset + size
        while pos + 12 <= end:
            namesz, descsz, n_type = struct.unpack_from('<3I', data, pos)
            name_at = pos + 12
            desc_at = name_at + (namesz + 3 & ~3)
            if n_type == NT_GNU_BUILD_ID and data[name_at:name_at + namesz] == b'GNU\0':
                return data[desc_at:desc_at + descsz].hex()
            pos = desc_at + (descsz + 3 & ~3)
    return None


def reply_value(reply, name):
    """Value of a `name=value` line in a CLI reply, or None."""
    for line in reply.splitlines():
        if line.startswith(name + '='):
            return line.split('=', 1)[1].strip()
    return None


def query_serial(port, baud, command, name, timeout):
    import serial
    with serial.Serial(port, baud, timeout=0.05, exclusive=True) as cli:
        cli.reset_input_buffer()
        cli.write((command + '\r\n').encode())
        reply, deadline = '', time.monotonic() + timeout
        while time.monotonic() < deadline:
            reply += cli.read(256).decode(errors='replace')
            if reply_value(reply, name) is not None or 'error=' in reply:
                break
        return reply


def board_build_id(args):
    from serial_console import default_socket_path, socket_request
    command = 'get build.id'
    path = Path(args.socket) if args.socket else default_socket_path(args.port)
    if args.socket or Path(path).exists():
        reply = socket_request(path, command)
    else:
        reply = query_serial(args.port, args.baud, command, 'build.id', args.timeout)
    value = reply_value(reply, 'build.id')
    if value is None:
        raise RuntimeError('no build.id in reply (firmware predates build.id?): %r' % reply)
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('port', nargs='?', help='CLI serial port')
    parser.add_argument('--socket', help='socket of a running serial_console.py')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--timeout', type=float, default=2.0)
    parser.add_argument('--elf', type=Path, action='append',
                        help='ELF to compare (repeatable); default: every firmware/.pio/build/*/firmware.elf')
    args = parser.parse_args(argv)
    if not args.port and not args.socket:
        parser.error('give a PORT or --socket')

    elves = args.elf or sorted(FIRMWARE_DIR.glob('.pio/build/*/firmware.elf'))
    if not elves:
        parser.error('no firmware.elf found; build first or pass --elf')
    try:
        board = board_build_id(args)
    except Exception as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 2

    print('board  %s' % board)
    status = 1
    for elf in elves:
        try:
            local = elf_build_id(elf.read_bytes())
        except (OSError, ValueError, struct.error) as exc:
            print('error  %s: %s' % (elf, exc), file=sys.stderr)
            return 2
        built = datetime.fromtimestamp(elf.stat().st_mtime).strftime('%Y-%m-%d %H:%M')
        mark = 'MATCH' if local == board else '     '
        if local == board:
            status = 0
        print('%s  %s  %s  (built %s)' % (mark, local or 'none', elf.parent.name if not args.elf else elf, built))
    if status:
        print('board image matches none of the builds above')
    return status


if __name__ == '__main__':
    sys.exit(main())

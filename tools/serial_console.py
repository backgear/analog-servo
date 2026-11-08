#!/usr/bin/env python3
"""Generic serial CLI owner: live TUI, firmware-discovered completion, local IPC.

python3 tools/serial_console.py PORT
python3 tools/serial_console.py PORT --headless --socket /tmp/servo.sock
python3 tools/serial_console.py --socket /tmp/servo.sock --send 'status'
"""
import argparse
from collections import deque
from concurrent.futures import Future, TimeoutError
from contextlib import contextmanager
import curses
import json
import math
import os
from pathlib import Path
import queue
import re
import socket
import threading
import time
import traceback
import unicodedata
import warnings


def text_prefix(values):
    """Common character prefix of completion strings; no filesystem semantics."""
    if not values:return ''
    first,last = min(values),max(values)
    for index,(left,right) in enumerate(zip(first,last)):
        if left!=right:return first[:index]
    return first


@contextmanager
def terminal_diagnostics(owner):
    """Keep asynchronous diagnostics inside the display while curses owns it."""
    previous_warning,previous_exception = warnings.showwarning,threading.excepthook
    def warning(message, category, filename, lineno, file=None, line=None):
        owner.record('warning',f'{category.__name__}: {message} ({filename}:{lineno})',False)
    def exception(args):
        owner.record('background error',''.join(traceback.format_exception(
            args.exc_type,args.exc_value,args.exc_traceback)),False)
    warnings.showwarning,threading.excepthook = warning,exception
    try:yield
    finally:warnings.showwarning,threading.excepthook = previous_warning,previous_exception


def clipped_text(value, cells):
    """Clip by terminal columns, dropping control characters from device output."""
    output,used = [],0
    for char in str(value):
        if not char.isprintable():continue
        width = 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in ('W','F') else 1)
        if used+width>cells:break
        if width==0 and not output:continue
        output.append(char);used += width
    return ''.join(output)


def draw_text(screen, row, column, value, attr=0, limit=None):
    height,width = screen.getmaxyx()
    if not (0<=row<height and 0<=column<width-1):return
    cells = width-column-1
    if limit is not None:cells = min(cells,max(0,limit))
    text = clipped_text(value,cells)
    if not text:return
    try:screen.addstr(row,column,text,attr)
    except curses.error:pass  # A resize can race with getmaxyx().


class Palette:
    def __init__(self, enabled=True):
        self.roles = dict(header=curses.A_REVERSE,disconnected=curses.A_REVERSE|curses.A_BOLD,
                          value=curses.A_BOLD,good=curses.A_BOLD,bad=curses.A_BOLD,
                          warning=curses.A_BOLD,command=curses.A_BOLD,hint=0,matches=curses.A_BOLD,prompt=curses.A_BOLD)
        if not enabled:return
        try:
            if not curses.has_colors():return
            curses.start_color()
            try:curses.use_default_colors();background = -1
            except curses.error:background = curses.COLOR_BLACK
            colors = [('header',curses.COLOR_WHITE,curses.COLOR_BLUE),
                      ('disconnected',curses.COLOR_WHITE,curses.COLOR_RED),
                      ('value',curses.COLOR_CYAN,background),('good',curses.COLOR_GREEN,background),
                      ('bad',curses.COLOR_RED,background),('warning',curses.COLOR_YELLOW,background),
                      ('command',curses.COLOR_CYAN,background),('hint',curses.COLOR_YELLOW,background),
                      ('matches',curses.COLOR_CYAN,background),('prompt',curses.COLOR_GREEN,background)]
            for index,(role,foreground,bg) in enumerate(colors,1):
                if index>=curses.COLOR_PAIRS:break
                curses.init_pair(index,foreground,bg)
                self.roles[role] = curses.color_pair(index) | (curses.A_BOLD if role!='hint' else 0)
        except curses.error:pass  # Monochrome/limited terminals keep usable attributes.

    def __getitem__(self, role):return self.roles[role]

    def value(self, key, value):
        if key.startswith('fault.') and key!='fault.reason':return self['bad'] if value not in ('0','?') else self['good']
        if key=='fault.reason':return self['bad'] if value not in ('none','?') else self['good']
        if key=='axis.enabled':return self['good'] if value=='1' else self['warning']
        return self['value']


class LineDecoder:
    """Accept real CRLF and firmware that transmits literal backslash-r-n."""
    def __init__(self):
        self.buffer = b''

    def feed(self, data):
        self.buffer += data.replace(b'\0', b'')
        if len(self.buffer) > 131072:
            raise ValueError('Unterminated serial line exceeds 128 KiB')
        lines = []
        while True:
            ends = [(i, n) for i, n in ((self.buffer.find(b'\n'), 1),
                                       (self.buffer.find(b'\\r\\n'), 4)) if i >= 0]
            if not ends:
                return lines
            i, n = min(ends)
            lines.append(self.buffer[:i].rstrip(b'\r').decode('utf-8', 'replace'))
            self.buffer = self.buffer[i+n:]


def key_values(lines):
    return dict(line.split('=', 1) for line in lines if '=' in line and not line.startswith(('error=','crc=')))


def usage_paths(description):
    """Extract literal prefixes from help such as <start relay <amp> | stop>."""
    text = description.split(' --', 1)[0].strip()
    if not text.startswith('<') or not text.endswith('>'):
        return []
    text = text[1:-1]
    branches, start, depth = [], 0, 0
    for i, ch in enumerate(text):
        if ch in '<[':depth += 1
        elif ch in '>]':depth -= 1
        elif ch == '|' and depth == 0:
            branches.append(text[start:i]);start = i+1
    branches.append(text[start:])
    paths = []
    for branch in branches:
        partial = [()]
        for token in re.findall(r'<[^<>]*>|\[[^\]]*\]|[^\s]+', branch):
            if token.startswith('<'):
                choices = token[1:-1].split('|')
                if len(choices) < 2:break
            elif token.startswith('['):break
            else:choices = [token]
            if not all(re.fullmatch(r'[A-Za-z_][\w-]*', c) for c in choices):break
            partial = [p+(c,) for p in partial for c in choices]
        paths.extend(partial)
    return paths


class Catalog:
    def __init__(self):
        self.commands = {}
        self.variables = {}
        self.max_command_bytes = 128
        self.tick_hz = None
        self.samples = {}

    def load_schema(self, document):
        if document.get('schema_version') != 1:
            raise ValueError('Unsupported schema version')
        commands, variables = document['commands'], document['variables']
        if not isinstance(commands, dict) or not isinstance(variables, dict):
            raise ValueError('Invalid command/variable schema')
        if not all(isinstance(k, str) and isinstance(v, str) for k,v in commands.items()):
            raise ValueError('Invalid command descriptions')
        if not all(isinstance(k, str) and isinstance(v, dict) for k,v in variables.items()):
            raise ValueError('Invalid variable metadata')
        self.commands = commands
        self.variables = variables
        self.max_command_bytes = int(document.get('max_command_bytes', 128))
        self.tick_hz = document.get('tick_hz')
        self.samples = document.get('samples',{})

    def hint(self, text, state):
        tokens = text.split()
        name = tokens[1] if len(tokens)>1 and tokens[0] in ('get','set') else text.strip()
        meta = self.variables.get(name)
        if meta is None:
            return self.commands.get(tokens[0], '') if tokens else 'Tab discovers commands; :refresh reloads metadata'
        raw = state.get(name, '?')
        bits = meta.get('fractional_bits', 0)
        display = raw
        if bits:
            try:display = f'{raw} / {2**bits} = {int(raw)/2**bits:g}'
            except (ValueError, TypeError):pass
        scale = f' · {bits} fractional bits' if bits else ''
        full = f" · full scale ±{meta['full_scale']}" if 'full_scale' in meta else ''
        choices = ' · '+', '.join(map(str,meta['choices'])) if meta.get('choices') else ''
        return f"{name}: {display} {meta.get('unit','')} · {meta.get('type','?')} {meta.get('access','?')}{scale}{full}{choices}"

    def complete(self, text, cursor):
        start = cursor
        while start>0 and not text[start-1].isspace():start -= 1
        end = cursor
        while end<len(text) and not text[end].isspace():end += 1
        prefix = text[:start].split()
        word = text[start:cursor]
        if not prefix:
            pool = [*self.commands, ':refresh', ':poll', ':snapshot', ':quit', ':clear']
        elif prefix[0] in ('get','set') and len(prefix)==1:
            pool = [k for k,v in self.variables.items() if prefix[0]=='get' or v.get('access')!='ro']
        elif prefix[0]=='set' and len(prefix)==2:
            pool = self.variables.get(prefix[1],{}).get('choices',[])
        else:
            pool = []
            for path in usage_paths(self.commands.get(prefix[0],'')):
                if tuple(prefix[1:]) == path[:len(prefix)-1] and len(path)>=len(prefix):
                    pool.append(path[len(prefix)-1])
        matches = sorted(set(str(v) for v in pool if str(v).lower().startswith(word.lower())))
        if not matches:matches = sorted(set(str(v) for v in pool if word.lower() in str(v).lower()))
        if not matches:return text,cursor,[]
        common = text_prefix(matches)
        if len(matches)==1:common = matches[0]
        if common and (len(common)>len(word) or len(matches)==1):
            suffix = ' ' if len(matches)==1 and end==len(text) else ''
            return text[:start]+common+suffix+text[end:],start+len(common)+len(suffix),matches
        return text,cursor,matches


class SerialOwner:
    def __init__(self, serial_port, poll_command='status', poll_interval=.25, timeout=3., checksum='none'):
        self.serial = serial_port
        self.poll_command, self.poll_interval = poll_command, poll_interval
        self.timeout, self.checksum = timeout, checksum
        self.catalog = Catalog()
        self.state, self.log = {}, deque(maxlen=1000)
        self.poll_keys = []
        self.lock = threading.RLock()
        self.jobs = queue.Queue(maxsize=64)
        self.stop = threading.Event()
        self.connected = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.decoder = LineDecoder()
        self.pending_lines = deque()
        self.last_poll = time.monotonic()

    def start(self):
        self.thread.start()
        return self

    def record(self, command, raw, ok=True, poll=False):
        with self.lock:
            self.log.append(dict(time=time.strftime('%H:%M:%S'),command=command,raw=raw,ok=ok,poll=poll))

    def snapshot(self):
        with self.lock:
            return dict(connected=self.connected,state=dict(self.state),commands=dict(self.catalog.commands),
                        variables=dict(self.catalog.variables),log=list(self.log))

    def submit(self, command, wait=10., poll=False):
        future = Future()
        if self.stop.is_set():
            future.set_exception(RuntimeError('Serial owner is stopped'));return future
        if '\n' in command or '\r' in command or '\0' in command or not command.strip():
            future.set_exception(ValueError('Send one nonempty command per request'));return future
        if len(command.encode())>self.catalog.max_command_bytes:
            future.set_exception(ValueError('Command exceeds firmware line limit'));return future
        try:self.jobs.put_nowait((command,future,time.monotonic()+wait,poll))
        except queue.Full:future.set_exception(RuntimeError('Command queue is full'))
        return future

    def send(self, command, wait=10.):
        future = self.submit(command,wait)
        try:return future.result(wait)
        except TimeoutError:
            future.cancel()
            raise RuntimeError('Request expired; an already-sent command may have executed')

    def read_line(self, deadline):
        while not self.stop.is_set() and time.monotonic()<deadline:
            if self.pending_lines:return self.pending_lines.popleft()
            data = self.serial.read(max(1,min(self.serial.in_waiting,4096)))
            if data:self.pending_lines.extend(self.decoder.feed(data))
        raise TimeoutError('Serial response timed out')

    def drain(self, duration=.06):
        """Record idle/late lines, never reset_input_buffer or retry a mutation."""
        until = time.monotonic()+duration
        lines = []
        while not self.stop.is_set() and time.monotonic()<until:
            try:line = self.read_line(until)
            except TimeoutError:break
            if line:lines.append(line)
        if lines:
            self.record('device', '\n'.join(lines))
            with self.lock:self.state.update(key_values(lines))

    def exchange(self, command, poll=False):
        self.drain()
        self.serial.write((command+'\r\n').encode());self.serial.flush()
        lines, deadline = [], time.monotonic()+self.timeout
        while True:
            line = self.read_line(deadline)
            if not line:break  # Empty response is a valid ACK (set/enable/etc.).
            lines.append(line)
        ok = not any(line.startswith('error=') for line in lines)
        if self.checksum=='xor8':
            body = [l for l in lines if not l.startswith('crc=')]
            trailers = [l[4:].strip().upper() for l in lines if l.startswith('crc=')]
            checksum = 0
            for value in ('\n'.join(body)+'\n').encode():checksum ^= value
            ok = ok and trailers==[f'{checksum:02X}']
        raw = '\n'.join(lines)+'\n\n'
        self.record(command,raw,ok,poll)
        if ok and command not in ('help','schema'):
            values = key_values(lines)
            with self.lock:
                for name, layout in self.catalog.samples.items():
                    if name in values:
                        fields = layout.get('fields',[])
                        parts = values.pop(name).split(',')
                        if len(parts)!=len(fields):raise ValueError('Sample field count disagrees with schema')
                        sample = dict(zip(fields,parts))
                        flags = int(sample.get('flags','0'))
                        for key, mask in layout.get('flags',{}).items():sample[key] = str(int(bool(flags & mask)))
                        values.update(sample)
                self.state.update(values)
                if poll:self.poll_keys = list(values)
        return dict(ok=ok,raw=raw,lines=lines)

    def discover(self):
        response = self.send('schema')
        try:
            if not response['ok']:raise ValueError('No schema command')
            document = json.loads('\n'.join(response['lines']))
            with self.lock:self.catalog.load_schema(document)
        except (ValueError, KeyError, TypeError) as exc:
            self.record('console',f'Schema unavailable ({exc}); falling back to help/settings')
            response = self.send('help')
            with self.lock:
                self.catalog.commands = dict(line.split(None,1) if ' ' in line else (line,'')
                                             for line in response['lines'] if line and not line.startswith(('error=','crc=')))
            settings = self.send('settings')
            with self.lock:self.catalog.variables = {key:{} for key in key_values(settings['lines']) if key!='crc'}
            if self.poll_command is None:self.poll_command = 'status'
            return
        self.send('settings')
        if self.poll_command is None:self.poll_command = 'sample' if 'sample' in self.catalog.samples else 'status'

    def run(self):
        try:
            while not self.stop.is_set():
                try:command,future,expiry,poll = self.jobs.get_nowait()
                except queue.Empty:
                    if self.poll_command and self.poll_interval and time.monotonic()-self.last_poll>=self.poll_interval:
                        command,future,expiry,poll = self.poll_command,Future(),math.inf,True
                        self.last_poll = time.monotonic()
                    else:
                        self.drain(.03);continue
                if future.cancelled():continue
                if time.monotonic()>expiry:
                    future.set_exception(RuntimeError('Queued request expired before send'));continue
                if not future.set_running_or_notify_cancel():continue
                try:
                    result = self.exchange(command,poll)
                    if poll and not result['ok']:
                        self.poll_interval = 0
                        self.record('console','Polling paused: choose --poll-command supported by this device',False)
                    future.set_result(result)
                except TimeoutError as exc:
                    self.record(command,str(exc),False,poll)
                    future.set_exception(RuntimeError(str(exc)))
                    # Quarantine late replies before the next queued request.
                    self.drain(self.timeout)
                except Exception as exc:
                    future.set_exception(exc);raise
        except Exception as exc:
            self.record('console',f'Disconnected: {exc}',False)
        finally:
            self.connected = False;self.stop.set()
            while not self.jobs.empty():
                _,future,_,_ = self.jobs.get_nowait()
                if not future.done():future.set_exception(RuntimeError('Serial connection closed'))

    def close(self):
        self.stop.set();self.thread.join(timeout=self.timeout+1)
        self.serial.close()


class CommandSocket:
    """Private local line protocol; serial I/O still belongs to SerialOwner."""
    def __init__(self, path, owner):
        self.path, self.owner = Path(path),owner
        self.server = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        self.stop = threading.Event()
        try:
            self.server.bind(str(self.path))  # Never unlink another owner's socket.
            os.chmod(self.path,0o600)
            self.server.listen(8);self.server.settimeout(.1)
        except Exception:
            self.server.close();raise
        self.thread = threading.Thread(target=self.run,daemon=True)

    def start(self):
        self.thread.start();return self

    def run(self):
        while not self.stop.is_set():
            try:connection,_ = self.server.accept()
            except socket.timeout:continue
            except OSError:break
            threading.Thread(target=self.handle,args=(connection,),daemon=True).start()

    def handle(self, connection):
        with connection:
            connection.settimeout(12)
            try:
                with connection.makefile('rb') as reader:
                    while not self.stop.is_set():
                        data = reader.readline(4098)
                        if not data:return
                        if len(data)>4096 or not data.endswith(b'\n'):
                            raise ValueError('Request must be a newline-terminated line under 4096 bytes')
                        command = data.decode().strip()
                        if command==':snapshot':reply = json.dumps(self.owner.snapshot())+'\n\n'
                        elif command==':refresh':
                            self.owner.discover();reply = 'console=discovery refreshed\n\n'
                        else:
                            result = self.owner.send(command)
                            reply = result['raw']
                            if not result['ok'] and not reply.startswith('error='):
                                reply = 'error=invalid response/checksum\n'+reply
                        connection.sendall(reply.encode())
            except Exception as exc:
                try:connection.sendall(('error=console: '+str(exc)+'\n\n').encode())
                except OSError:pass

    def close(self):
        self.stop.set();self.server.close();self.thread.join(timeout=1)
        self.path.unlink(missing_ok=True)


def socket_request(path, command):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(15);connection.connect(str(path))
        connection.sendall((command+'\n').encode())
        response = bytearray()
        while b'\n\n' not in response:
            chunk = connection.recv(65536)
            if not chunk:break
            response.extend(chunk)
        return response.decode()


def tui(screen, owner, path, watch, colors=True):
    palette = Palette(colors)
    previous_size = None
    screen.keypad(True);screen.timeout(100)
    try:curses.curs_set(1)
    except curses.error:pass
    text,cursor,matches = '',0,[]
    history,index,draft = [],0,''
    show_poll,scroll = False,0
    while True:
        snap = owner.snapshot()
        height,width = screen.getmaxyx()
        if previous_size!=(height,width):screen.clearok(True);previous_size = (height,width)
        screen.erase()
        def put(y,value,attr=0):draw_text(screen,y,0,value,attr)
        put(0,f" Serial console · {owner.serial.port} · {'connected' if snap['connected'] else 'DISCONNECTED'} · {path}",
            palette['header' if snap['connected'] else 'disconnected'])
        with owner.lock:keys = watch or list(owner.poll_keys)
        # Generic live panel: poll-discovered keys, or explicit --watch names.
        panel = []
        for key in keys:
            meta = snap['variables'].get(key,{})
            value = snap['state'].get(key,'?')
            unit = meta.get('unit','')
            panel.append((f"{key} = {value}"+(f" [{unit}]" if unit else ''),palette.value(key,value)))
        rows = min(max(1,height//4),len(panel))
        columns = max(1,width//40)
        for i,(value,attr) in enumerate(panel[:rows*columns]):
            y,x = 2+i//columns,(i%columns)*(max(1,width//columns))
            draw_text(screen,y,x,value,attr,limit=width//columns-1)
        top = 3+min(rows,math.ceil(len(panel)/columns))
        lines = []
        for event in snap['log']:
            if event['poll'] and not show_poll:continue
            if event['command'] in ('schema','settings') and event['ok']:
                lines.append((f"{event['time']} > {event['command']}  [{len(event['raw'])} bytes; metadata/state updated]",palette['command']))
                continue
            attr = palette['command' if event['ok'] else ('warning' if event['command']=='warning' else 'bad')]
            lines.append((f"{event['time']} {'>' if event['ok'] else '!'} {event['command']}",attr))
            lines.extend(('  '+line,0 if event['ok'] else attr) for line in event['raw'].strip().splitlines())
        available = max(0,height-top-4)
        end = max(0,len(lines)-scroll)
        for i,(line,attr) in enumerate(lines[max(0,end-available):end]):put(top+i,line,attr)
        with owner.lock:hint = owner.catalog.hint(text,snap['state'])
        put(height-4,hint,palette['hint'])
        put(height-3,'Matches: '+', '.join(matches) if matches else 'Tab complete · ↑↓ history · PgUp/PgDn log · Ctrl-P polls · Ctrl-L redraw · Ctrl-C quit',palette['matches'] if matches else 0)
        put(height-2,'No command retries. :refresh metadata · :poll toggle · :snapshot · :clear · :quit')
        offset = max(0,cursor-max(1,width-4))
        put(height-1,'> '+text[offset:],palette['prompt'])
        try:screen.move(height-1,min(width-1,2+cursor-offset))
        except curses.error:pass
        screen.refresh()
        try:key = screen.get_wch()
        except curses.error:continue
        if key in ('\x03','\x04'):return
        if key in ('\n','\r'):
            command = text.strip()
            if command:
                history.append(command);index = len(history);draft = '';scroll = 0
                if command==':quit':return
                if command==':refresh':
                    threading.Thread(target=owner.discover,daemon=True).start()
                elif command==':poll':owner.poll_interval = 0 if owner.poll_interval else .25
                elif command==':clear':
                    with owner.lock:owner.log.clear()
                elif command==':snapshot':owner.record('snapshot',json.dumps(snap['state'],indent=2))
                else:
                    future = owner.submit(command)
                    def report(f):
                        try:f.result()
                        except Exception as exc:owner.record('console',str(exc),False)
                    future.add_done_callback(report)
            text,cursor,matches = '',0,[]
        elif key=='\t':
            with owner.lock:text,cursor,matches = owner.catalog.complete(text,cursor)
            screen.clearok(True)
        elif key in (curses.KEY_BACKSPACE,'\x7f','\b'):
            if cursor:text = text[:cursor-1]+text[cursor:];cursor -= 1
            matches = []
        elif key==curses.KEY_DC:text = text[:cursor]+text[cursor+1:]
        elif key in (curses.KEY_LEFT,'\x02'):cursor = max(0,cursor-1)
        elif key in (curses.KEY_RIGHT,'\x06'):cursor = min(len(text),cursor+1)
        elif key in (curses.KEY_HOME,'\x01'):cursor = 0
        elif key in (curses.KEY_END,'\x05'):cursor = len(text)
        elif key=='\x15':text,cursor = text[cursor:],0
        elif key in (curses.KEY_UP,curses.KEY_DOWN):
            if index==len(history):draft = text
            index = max(0,min(len(history),index+(-1 if key==curses.KEY_UP else 1)))
            text = history[index] if index<len(history) else draft
            cursor = len(text);matches = []
        elif key==curses.KEY_PPAGE:scroll += max(1,available//2)
        elif key==curses.KEY_NPAGE:scroll = max(0,scroll-max(1,available//2))
        elif key=='\x10':show_poll = not show_poll
        elif key=='\x0c':screen.clearok(True)
        elif isinstance(key,str) and key.isprintable():
            text = text[:cursor]+key+text[cursor:];cursor += 1;matches = []


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('port',nargs='?')
    parser.add_argument('--baud',type=int,default=115200)
    parser.add_argument('--poll',type=float,default=.25,help='Idle status interval; 0 disables')
    parser.add_argument('--poll-command',default=None,help='Default: schema sample when available, otherwise status')
    parser.add_argument('--timeout',type=float,default=3.)
    parser.add_argument('--checksum',choices=('none','xor8'),default='none')
    parser.add_argument('--socket',dest='socket_path')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--no-color',action='store_true',default='NO_COLOR' in os.environ,help='Use monochrome display (also respects NO_COLOR)')
    parser.add_argument('--watch',nargs='+',help='Live panel variable names; otherwise uses status fields')
    parser.add_argument('--send',metavar='COMMAND',help='Send through an already-running console socket')
    parser.add_argument('--snapshot',action='store_true',help='Read running owner state as JSON')
    parser.add_argument('--list-ports',action='store_true')
    args = parser.parse_args(argv)
    if args.send is not None or args.snapshot:
        if not args.socket_path:parser.error('--send/--snapshot requires --socket')
        reply = socket_request(args.socket_path,':snapshot' if args.snapshot else args.send)
        print(reply,end='')
        return 1 if reply.startswith('error=') else 0
    if not math.isfinite(args.poll) or args.poll<0 or not math.isfinite(args.timeout) or args.timeout<=0:
        parser.error('--poll must be finite and nonnegative; --timeout must be finite and positive')
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:parser.error('Install pyserial: python3 -m pip install -r tools/requirements-console.txt')
    if args.list_ports:
        for port in list_ports.comports():print(f'{port.device}\t{port.description}\t{port.hwid}')
        return 0
    if not args.port:parser.error('Choose a CLI port (see --list-ports)')
    path = args.socket_path or f'/tmp/serial-console-{os.getuid()}-{Path(args.port).name}.sock'
    owner = SerialOwner(serial.Serial(args.port,args.baud,timeout=.03,write_timeout=1,exclusive=True),
                        args.poll_command,args.poll,args.timeout,args.checksum)
    server = None
    result = 0
    try:
        server = CommandSocket(path,owner).start()
        owner.start()
        owner.discover()
        print(f'Serial owner ready: {path}',flush=True)
        if args.headless:
            while not owner.stop.wait(.2):pass
            result = 1
        else:
            with terminal_diagnostics(owner):
                curses.wrapper(tui,owner,path,args.watch,not args.no_color)
    except KeyboardInterrupt:pass
    finally:
        if server:server.close()
        if owner.thread.ident is not None:owner.close()
        else:owner.serial.close()
    return result


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,RuntimeError,ValueError) as exc:raise SystemExit(str(exc))

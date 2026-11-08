"""Private local IPC for steering the live console without reopening serial."""
import json
import os
from pathlib import Path
import socket
import tempfile
import threading

ACTIONS={'quit','hold','velocity','move','gain','stop','disable','mark','pin','restore','refresh','save','clear'}

class ControlSocket:
    def __init__(self,session):
        self.session=session
        self.directory=Path(tempfile.mkdtemp(prefix='servo-scorecard-'))
        self.path=self.directory/'control.sock'
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        try:
            self.socket.bind(str(self.path));os.chmod(self.path,0o600)
            self.socket.listen(4);self.socket.settimeout(.1)
        except Exception:
            self.socket.close();self.path.unlink(missing_ok=True);self.directory.rmdir();raise
        self.stop=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True)
    def start(self):
        self.thread.start();return self
    def run(self):
        while not self.stop.is_set():
            try:conn,_=self.socket.accept()
            except socket.timeout:continue
            except OSError:return
            with conn:
                conn.settimeout(1)
                try:
                    data=bytearray()
                    while b'\n' not in data and len(data)<4096:
                        chunk=conn.recv(4096-len(data))
                        if not chunk:break
                        data.extend(chunk)
                    request=json.loads(data.split(b'\n')[0])
                    action=request['action']
                    if action=='snapshot':
                        reply=dict(score=self.session.model.snapshot(),settings=dict(self.session.settings),messages=list(self.session.messages))
                    elif action in ACTIONS:
                        self.session.recorder.event('request',action=action,value=request.get('value'),seq=self.session.model.last_seq)
                        self.session.request(action,request.get('value'))
                        reply={'queued':True,'action':action,'seq':self.session.model.last_seq}
                    else:raise ValueError('Action is not allowed')
                except Exception as exc:reply={'error':str(exc)}
                try:conn.sendall((json.dumps(reply,default=str)+'\n').encode())
                except OSError:pass
    def close(self):
        self.stop.set();self.socket.close();self.thread.join(timeout=2)
        self.path.unlink(missing_ok=True)
        try:self.directory.rmdir()
        except OSError:pass


def send(path,action,value=None):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
        s.settimeout(3);s.connect(str(path));s.sendall((json.dumps({'action':action,'value':value})+'\n').encode())
        data=bytearray()
        while b'\n' not in data:
            chunk=s.recv(65536)
            if not chunk:break
            data.extend(chunk)
    return json.loads(data.split(b'\n')[0])

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('socket');p.add_argument('action',choices=sorted(ACTIONS|{'snapshot'}));p.add_argument('value',nargs='?');a=p.parse_args()
    value=a.value
    if value is not None:
        try:value=json.loads(value)
        except ValueError:pass
    print(json.dumps(send(a.socket,a.action,value),indent=2))

"""Live bench metrics, provisional criteria and durable session recording.
No GUI or device dependencies: usable for replay and unit tests.
"""
from collections import deque
from dataclasses import dataclass, asdict
from pathlib import Path
import csv
import json
import math
import threading
import time

@dataclass
class Criteria:
    speed_percent: float = 1.0
    position_error: float = 100.0
    saturation_percent: float = 5.0
    saturation_hold_ms: float = 250.0
    overspeed: float = 80.0
    stale_seconds: float = 0.5
    torque_limit: float = 10000.0
    warmup_seconds: float = 1.0
    polarity_sign: int = -1
    cpr: int = 4096  # decoded quadrature edges: bench encoder is 1024 cycles x4
    tick_hz: int = 1000

class Scorecard:
    def __init__(self, criteria=None):
        self.criteria = criteria or Criteria()
        self.frames = deque(maxlen=self.criteria.tick_hz * 60)
        self.last_received = None
        self.last_seq = None
        self.missing = 0
        self.resets = 0
        self._target = None
        self._stable_since = 0
        self._stable_seq = 0
        self._sat_seq = None
        self._sat_since = None
        self.baseline = None
        self.move = None
        self.lock = threading.RLock()

    def add(self, frame, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            seq = frame['seq']
            if self.last_seq is not None:
                delta = (seq-self.last_seq) & 0xffffffff
                if delta == 0:
                    return
                if delta > 0x7fffffff:
                    self.frames.clear()
                    self.resets += 1
                    self._target = None
                elif delta > 1:
                    self.missing += delta-1
            if frame['cmd_velocity'] != self._target:
                self._target = frame['cmd_velocity']
                self._stable_since = now
                self._stable_seq = seq
            saturated = abs(frame['torque']) >= self.criteria.torque_limit * .999
            if not saturated:
                self._sat_since = None
                self._sat_seq = None
            elif self._sat_since is None:
                self._sat_since = now
                self._sat_seq = seq
            self._update_move(frame)
            self.frames.append(dict(frame, received=now))
            self.last_received, self.last_seq = now, seq

    def _update_move(self, frame):
        if not self.move:return
        m=self.move
        if ((frame['seq']-m['start_seq'])&0xffffffff)>0x7fffffff:return
        error=m['target']-frame['position']
        m['error']=error
        m['overshoot']=max(m['overshoot'],max(0,-m['direction']*error))
        if abs(error)>5:
            m['band_start']=None;m['settle_ms']=None
        elif m['band_start'] is None:
            m['band_start']=frame['seq']
        elif ((frame['seq']-m['band_start'])&0xffffffff)>=self.criteria.tick_hz*.05:
            m['settle_ms']=((m['band_start']-m['start_seq'])&0xffffffff)*1000/self.criteria.tick_hz

    def begin_move(self, target, start_position, start_seq):
        with self.lock:
            self.move=dict(target=target,start_seq=start_seq,direction=1 if target>=start_position else -1,
                error=target-start_position,overshoot=0,band_start=None,settle_ms=None)
            for f in self.frames:self._update_move(f)

    def cancel_move(self):
        with self.lock:self.move=None

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.frames:
                return {'state':'WAITING', 'rows':[], 'stale':True, 'guard':None, 'latest':None}
            latest = self.frames[-1]
            age = now-self.last_received
            # Firmware time anchors each window, avoiding GUI-rate sampling bias.
            window=[]
            for f in reversed(self.frames):
                if ((latest['seq']-f['seq']) & 0xffffffff)>=self.criteria.tick_hz:break
                window.append(f)
            window.reverse()
            short = [f for f in window if ((latest['seq']-f['seq']) & 0xffffffff) < self.criteria.tick_hz//10]
            speed = sum(f['velocity'] for f in window)/len(window)
            short_speed = sum(f['velocity'] for f in short)/len(short)
            target = latest['cmd_velocity']
            error = speed-target
            percent = abs(error)/max(abs(target),1)*100
            rms = math.sqrt(sum((f['velocity']-target)**2 for f in window)/len(window))
            position_peak = max(abs(f['cmd_position']-f['position']) for f in window)
            sat = sum(abs(f['torque'])>=self.criteria.torque_limit*.999 for f in window)/len(window)*100
            torque_mean = sum(self.criteria.polarity_sign*f['torque'] for f in window)/len(window)  # bench polarity=-1
            span = ((latest['seq']-window[0]['seq']) & 0xffffffff)+1
            coverage = min(100.0,len(window)/span*100)
            settling = ((latest['seq']-self._stable_seq)&0xffffffff) < self.criteria.warmup_seconds*self.criteria.tick_hz or span < self.criteria.tick_hz
            stale = age > self.criteria.stale_seconds
            guard = None
            if latest['fault_latched']:
                guard = 'Firmware fault: '+latest['fault_reason']
            elif latest['enabled'] and stale:
                guard = 'Telemetry stale'
            elif latest['enabled'] and abs(latest['velocity']) > self.criteria.overspeed:
                guard = 'Measured overspeed'
            elif latest['enabled'] and self._sat_since is not None and ((latest['seq']-self._sat_seq)&0xffffffff)*1000/self.criteria.tick_hz > self.criteria.saturation_hold_ms:
                guard = 'Sustained torque saturation'
            rows = [
                ('Telemetry coverage ≥99.9%', coverage>=99.9 and not stale, f'{coverage:.2f}% / age {age*1000:.0f} ms'),
                (f'Mean speed error ≤{self.criteria.speed_percent:g}%', percent<=self.criteria.speed_percent, f'{error:+.3f} ct/tick ({percent:.2f}%)'),
                (f'Following error ≤{self.criteria.position_error:g} ct', position_peak<=self.criteria.position_error, f'{position_peak:.0f} counts'),
                (f'Torque saturation ≤{self.criteria.saturation_percent:g}%', sat<=self.criteria.saturation_percent, f'{sat:.1f}% of last second'),
                ('Firmware faults', not latest['fault_latched'], latest['fault_reason']),
            ]
            if self.move:
                m=self.move
                detail=f"err {m['error']:+d} · over {m['overshoot']} · settle {m['settle_ms'] if m['settle_ms'] is not None else '…'} ms"
                rows[1]=('Endpoint ≤5 ct / stable 50 ms',abs(m['error'])<=5 and m['settle_ms'] is not None,detail)
            if not latest['enabled']:
                rows[1]=('Speed tracking',None,'Inactive while disabled')
                rows[2]=('Following error',None,'Target diagnostic is stale while disabled')
            if guard:
                state = 'STOPPED / '+guard
            elif stale:
                state = 'STALE'
            elif not latest['enabled']:
                state = 'DISABLED'
            elif self.move:
                state = 'POINT SETTLED' if self.move['settle_ms'] is not None else 'POINT MOVING'
            elif target == 0:
                state = 'HOLDING'
            elif settling:
                state = 'SETTLING'
            else:
                state = 'BENCH PASS' if all(r[1] for r in rows) else 'NEEDS WORK'
            return dict(state=state,rows=rows,stale=stale,guard=guard,latest=latest,
                mean_speed=speed,mean100ms_speed=short_speed,target=target,
                rpm=speed*self.criteria.tick_hz*60/self.criteria.cpr,
                target_rpm=target*self.criteria.tick_hz*60/self.criteria.cpr,
                speed_rmse=rms,position_peak=position_peak,saturation_percent=sat,
                torque_mean=torque_mean,missing=self.missing,resets=self.resets,
                criteria=asdict(self.criteria),move=dict(self.move) if self.move else None)

    def pin(self):
        with self.lock:
            self.baseline = self.snapshot()
            return self.baseline

    def history(self, seconds=5):
        with self.lock:
            if not self.frames:
                return []
            last = self.frames[-1]['seq']
            values=[]
            for f in reversed(self.frames):
                if ((last-f['seq'])&0xffffffff)>=seconds*self.criteria.tick_hz:break
                values.append(dict(f))
            values.reverse();return values

class Recorder:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True,exist_ok=False)
        self.raw = (self.directory/'monitor.bin').open('wb')
        self.events = (self.directory/'events.jsonl').open('w')
        self.csv_file = (self.directory/'monitor.csv').open('w',newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['seq','position','velocity','physical_torque','reference_position','ramped_velocity','enabled','fault'])
        self.lock = threading.RLock()
        self.closed = False

    def event(self, kind, **values):
        with self.lock:
            if self.closed:return
            self.events.write(json.dumps(dict(kind=kind,utc=time.time(),monotonic=time.monotonic(),**values))+'\n')
            self.events.flush()

    def write(self, data, frames):
        with self.lock:
            if self.closed:return
            self.raw.write(data)
            for f in frames:
                self.csv.writerow([f['seq'],f['position'],f['velocity'],f['torque'],f['cmd_position'],f['cmd_velocity'],int(f['enabled']),f['fault_reason']])

    def close(self, summary):
        with self.lock:
            if self.closed:return
            (self.directory/'summary.json').write_text(json.dumps(summary,indent=2,default=str)+'\n')
            for f in (self.raw,self.events,self.csv_file):f.close()
            self.closed = True

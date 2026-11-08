"""One owner for both serial ports, command logging and independent guards."""
import heapq
import itertools
import json
import math
import re
import threading
import time

import serial
from monitor_frame import FrameSync
from scorecard_core import Scorecard

# Unattended autotune campaign: relay plant-ID at an escalating amplitude ladder (bounded by
# the configured torque limit) until it finds oscillation, then a host-driven coordinate-descent
# refine over real scored validation moves -- the firmware only ever had a one-shot relay method
# (see Autotune.c); everything past "find Ku/Pu once" is orchestrated here, where there's room to
# log, bound and inspect every trial instead of trusting an opaque embedded search.
#
# Starts at 1%, not a higher naive guess: the amplitude a plant actually needs to oscillate
# can be well below what looks like a reasonable starting point (as low as ~3% of the
# configured torque limit on some plants), so the ladder has to sweep down into the low
# single-digit percent range to find it, not just start there as a formality.
AUTOTUNE_AMP_STEPS = (0.01, 0.02, 0.035, 0.05, 0.07, 0.09, 0.12, 0.2, 0.32, 0.5, 0.8, 1.0)   # fraction of torque.max, tried in order until relay reports 'ok'
# Tiered per-rung timeout, same length/indexing as AUTOTUNE_AMP_STEPS: short for the low rungs
# (a working amplitude shows oscillation within a few seconds or clearly doesn't -- no reason
# to sit through a full 6s of near-silent, imperceptible torque before escalating) and longer
# once amplitude is high enough that a miss is more consequential and worth more patience
# before escalating further.
AUTOTUNE_PROBE_MAX_TICKS_STEPS = (2500, 2500, 2500, 2500, 2500, 2500, 6000, 6000, 6000, 6000, 6000, 6000)
AUTOTUNE_REFINE_ROUNDS = 3        # bounded coordinate-descent rounds over (pos_p, vel_kp, vel_ki)
AUTOTUNE_REFINE_STEP = 0.25       # initial +-25% nudge per coordinate, halved every full round
AUTOTUNE_REFINE_MIN_STEP_BASE = 64   # see _nudge -- a percentage-of-tiny-value step can't climb
                                      # out of single digits; this is the firmware's own
                                      # conservative CONFIG_DEFAULT_POS_GAIN_Q8/VEL_GAIN_KP_Q8
                                      # scale, used as the absolute-step floor, shrunk by the
                                      # same per-round schedule as the percentage step.
AUTOTUNE_REFINE_COORDS = ('position.kp', 'velocity.kp', 'velocity.ki')
AUTOTUNE_TRIAL_REPEATS = 2         # real validation moves averaged per candidate (baseline and
                                   # every refine trial) -- real measurement noise alone (e.g.
                                   # the same candidate's Ki landing anywhere from 646-734,
                                   # settle time 425-720ms, across otherwise-identical trials)
                                   # is big enough to flip an accept/reject decision on a
                                   # single sample.
AUTOTUNE_MIN_IMPROVEMENT_FRACTION = 0.05   # a refine candidate must beat the current best by at
                                            # least this fraction, not just any nonzero amount --
                                            # stops the search from "accepting" a change that's
                                            # really just noise.
AUTOTUNE_PASS_SETTLE_MS = 550        # Not a safety limit -- purely a reporting threshold (PASS
                                      # vs CHECK); gains are applied either way, this only
                                      # decides whether the campaign trusts the result or flags
                                      # it for a look. 550ms allows a clean, well-damped response
                                      # that settles a little slower than 400ms to still pass,
                                      # since settle time alone doesn't indicate instability the
                                      # way overshoot does.
AUTOTUNE_PASS_OVERSHOOT_MIN = 10     # counts -- the tighter of the two gates, since overshoot is
                                      # the number that actually signals ringing/instability
                                      # tendency, not just "not fast enough".
AUTOTUNE_PASS_OVERSHOOT_FRACTION = 0.08   # of the validation move distance
AUTOTUNE_OVERSHOOT_COST_WEIGHT = 15   # ct-to-ms exchange rate inside _trial_cost's search
                                       # objective. A low weight lets the search trade a large
                                       # chunk of settle time for real ringing (e.g. 257ms/114ct,
                                       # ~22% of a 512ct move) because the apparent win in
                                       # settle-ms dwarfs such a small per-count penalty. Derived
                                       # from the PASS thresholds themselves, not picked blind:
                                       # at a 512ct move, the overshoot pass bound (41ct) times
                                       # this weight (~615) lands in the same ballpark as the
                                       # settle pass bound (550ms), so blowing the overshoot
                                       # bound costs the search about as much as blowing the
                                       # whole settle budget, instead of being a footnote next
                                       # to it.

class LiveSession:
    def __init__(self, monitor_port, cli_port, recorder, criteria, baud=2500000, cli_baud=115200, max_command=40,
                 move_settle_threshold=1):
        self.model = Scorecard(criteria)
        self.recorder = recorder
        self.monitor_port, self.cli_port = monitor_port, cli_port
        self.baud, self.cli_baud = baud, cli_baud
        self.max_command = max_command
        # 'move' refuses to start while |mean100ms_speed| exceeds this -- the default of 1
        # assumes a bench that actually comes to rest; a drive with persistent ambient drift
        # (never settling near zero) needs this raised, or no move ever gets issued at all.
        self.move_settle_threshold = move_settle_threshold
        self.settings = {}
        self.saved_settings = {}
        self.messages = []
        self.shutdown_requested = False
        self.requested_speed = 0
        self.hold_deadline = None
        self.hold_start_seq = None
        self.autotune_phase = None   # None | 'probing' | 'applying' | 'settling' | 'measuring'
        self.autotune_verdict = None
        self.autotune_log = []
        self._autotune_ctx = None
        self._autotune_poll_at = 0
        self._autotune_deadline = 0
        self.running = threading.Event()
        self._queue = []
        self._condition = threading.Condition()
        self._counter = itertools.count()
        self._generation = 0
        self._threads = []
        self._guarded = False
        self._fault_seen = False
        self._settings_ready = False
        self._reset_seen = 0
        self._cli = self._monitor = None
        self.control_socket = None
        self._settings_lock = threading.RLock()

    def start(self):
        # Opening is read-only until the owner thread explicitly starts telemetry.
        self._monitor = serial.Serial(self.monitor_port,self.baud,timeout=.03,write_timeout=.5)
        try:
            if self.cli_port:self._cli = serial.Serial(self.cli_port,self.cli_baud,timeout=.03,write_timeout=.5)
        except Exception:
            self._monitor.close()
            raise
        self.running.set()
        from scorecard_control import ControlSocket
        try:self.control_socket=ControlSocket(self).start()
        except Exception:
            self.running.clear();self._monitor.close()
            if self._cli:self._cli.close()
            raise
        (self.recorder.directory/'connection.json').write_text(json.dumps({'control_socket':str(self.control_socket.path),'monitor_port':self.monitor_port,'cli_port':self.cli_port},indent=2)+'\n')
        for target in (self._read_monitor,self._control):
            t=threading.Thread(target=target,daemon=True);t.start();self._threads.append(t)
        return self

    def note(self, message):
        self.messages.append(str(message));self.messages=self.messages[-8:]
        try:self.recorder.event('notice',message=str(message),seq=self.model.last_seq)
        except OSError:self.messages.append('Recording failed; stop and check storage')

    def request(self, action, value=None):
        if not self.cli_port and action not in ('mark','pin'):
            self.note('Read-only: connect a CLI port for controls');return
        with self._condition:
            priority=0 if action in ('disable','stop','shutdown','quit') else 10
            if priority==0:
                self.hold_deadline=None
                self._generation+=1
                self._queue.clear()  # queued starts cannot execute after a stop
            heapq.heappush(self._queue,(priority,next(self._counter),self._generation,action,value))
            self._condition.notify()

    def _exchange(self, command):
        if not self._cli:return ''
        sent_seq=self.model.last_seq
        self.last_command_seq=sent_seq
        self._cli.write((command+'\r\n').encode());self._cli.flush()
        data=bytearray();deadline=time.monotonic()+1.2;last_data=None
        while time.monotonic()<deadline:
            chunk=self._cli.read(max(1,self._cli.in_waiting))
            if chunk:data.extend(chunk);last_data=time.monotonic()
            elif last_data is not None and time.monotonic()-last_data>.06:break
        response=data.decode(errors='replace').replace('\\r\\n','\n')
        self.recorder.event('command',command=command,response=response,seq=sent_seq,reply_seq=self.model.last_seq)
        if not data:raise RuntimeError('No CLI response: '+command)
        if re.search(r'(?m)^error=', response):raise RuntimeError(response.strip())
        if command.startswith('set '):
            _,key,value=command.split(maxsplit=2)
            with self._settings_lock:self.settings[key]=value
        if command=='settings':
            parsed=dict(line.split('=',1) for line in response.splitlines() if '=' in line)
            with self._settings_lock:
                self.settings.update(parsed)
                if 'output.polarity' in parsed:self.model.criteria.polarity_sign=int(parsed['output.polarity'])
                if 'torque.min' in parsed:
                    self.model.criteria.torque_limit=max(abs(int(parsed['torque.min'])),abs(int(parsed['torque.max'])))
            self._settings_ready=True
        return response

    def _sequence(self, commands, generation):
        for command in commands:
            if generation!=self._generation:return
            self._exchange(command)

    def _guard(self):
        if self.hold_deadline is not None and time.monotonic()>=self.hold_deadline:
            score=self.model.snapshot()
            values=[f for f in self.model.history(60) if ((f['seq']-self.hold_start_seq)&0xffffffff)>=self.model.criteria.tick_hz and ((f['seq']-self.hold_start_seq)&0xffffffff)<0x7fffffff]
            summary=dict(samples=len(values),mean_speed=sum(f['velocity'] for f in values)/len(values) if values else None,peak_torque=max((abs(f['torque']) for f in values),default=0),peak_following_error=max((abs(f['cmd_position']-f['position']) for f in values),default=0))
            self.recorder.event('hold_complete',score=score,steady=summary,settings=dict(self.settings),seq=self.model.last_seq)
            self.note('Timed hold complete; stopping')
            self.request('stop')
        snap=self.model.snapshot()
        if snap['guard'] and not self._guarded:
            self._guarded=True
            self.note('Guard stop: '+snap['guard']);self.request('disable')
        if self.model.resets!=self._reset_seen:
            self._reset_seen=self.model.resets;self.note('Device tick reset/history discontinuity');self.request('disable');self.request('refresh')
        latest=snap['latest']
        if latest and latest['fault_latched'] and not self._fault_seen:
            self._fault_seen=True
            try:self._exchange('fault freeze')
            except Exception as exc:self.note(exc)

    def autotune_status(self):
        """Human status line for the GUI -- keeps _autotune_ctx's shape private to this class."""
        if self.autotune_phase is None:return self.autotune_verdict or 'Autotune: idle'
        ctx=self._autotune_ctx
        if self.autotune_phase=='probing':
            return f"Probing amplitude {round(ctx['torqueClampMax']*AUTOTUNE_AMP_STEPS[ctx['amp_index']])} ({ctx['amp_index']+1}/{len(AUTOTUNE_AMP_STEPS)})"
        repeat=f" (repeat {len(ctx['repeat_costs'])+1}/{AUTOTUNE_TRIAL_REPEATS})" if ctx['purpose'] else ''
        if ctx['purpose']=='baseline':return 'Applied relay result; scoring baseline…'+repeat
        coord=AUTOTUNE_REFINE_COORDS[ctx['coord_index']].rsplit('.',1)[-1]
        return f"Refining round {ctx['round']+1}/{AUTOTUNE_REFINE_ROUNDS} · {coord} {ctx['trial_kind']}"+repeat

    def _trial_cost(self, move, failed):
        # Unsettled/failed trials always lose to any settled one; among settled trials, weight
        # overshoot above raw settle time -- a fast-but-overshooting move is worse than a
        # slightly-slower clean one for a servo, not just "whichever number is smaller."
        if failed or move is None or move.get('settle_ms') is None:
            return 1e9+(abs(move['overshoot']) if move else 0)
        return move['settle_ms']+abs(move['overshoot'])*AUTOTUNE_OVERSHOOT_COST_WEIGHT

    def _verdict_label(self, move):
        if move is None or move.get('settle_ms') is None:return 'CHECK'
        distance=self._autotune_ctx['distance'] if self._autotune_ctx else move.get('target',0)
        return 'PASS' if move['overshoot']<=max(AUTOTUNE_PASS_OVERSHOOT_MIN,distance*AUTOTUNE_PASS_OVERSHOOT_FRACTION) and move['settle_ms']<=AUTOTUNE_PASS_SETTLE_MS else 'CHECK'

    def _nudge(self, value, fraction, direction):
        # A percentage step is useless starting from a tiny value: relay's own pos_p heuristic
        # ("outer loop ~5x slower than the tuned velocity loop") can land as low as 1-2, and
        # round(1*0.25)==0 means every "up"/"down" trial would be a +-1 step forever -- 3 rounds
        # can't climb anywhere from there. AUTOTUNE_REFINE_MIN_STEP_BASE (scaled by the same
        # shrinking step_fraction schedule everything else uses) gives it room to actually move;
        # for vel_kp/vel_ki, already in the tens of thousands, the percentage term dominates and
        # this is a no-op. Floored at 0 either way -- a negative gain is positive feedback, not a
        # worse tune (Protocol.c's setGainsPosP/VelKp/VelKi reject it too, belt and suspenders).
        value=int(value)
        min_step=max(1,round(AUTOTUNE_REFINE_MIN_STEP_BASE*fraction/AUTOTUNE_REFINE_STEP))
        step=32 if value==0 else max(1,round(abs(value)*fraction),min_step)   # 32: a fixed absolute step off zero, since any percentage of zero is zero
        return max(0,value+step*direction)

    def _autotune_tick(self):
        # One-button campaign: relay plant-ID at an escalating amplitude -> apply -> host-driven
        # coordinate-descent refine over real scored moves -> pass/fail verdict. Polls via
        # 'settings' (same full-registry refresh path used everywhere else, including read-only
        # diagnostic fields like autotune.result) rather than the firmware's unsolicited
        # 'autotune.progress' lines, which would race with _exchange()'s synchronous request/
        # response framing.
        if self.autotune_phase is None:return
        now=time.monotonic()
        if now<self._autotune_poll_at:return
        self._autotune_poll_at=now+0.3
        ctx=self._autotune_ctx
        if self.autotune_phase=='probing':
            try:self._exchange('settings')
            except Exception as exc:self._autotune_finish('Autotune poll failed: '+str(exc));return
            if self.settings.get('autotune.active')=='1':return
            result=self.settings.get('autotune.result','?')
            amplitude=round(ctx['torqueClampMax']*AUTOTUNE_AMP_STEPS[ctx['amp_index']])
            if result=='ok':
                ctx['gains']['position.kp']=int(self.settings.get('autotune.pos_p_result',0))
                ctx['gains']['velocity.kp']=int(self.settings.get('autotune.kp_result',0))
                ctx['gains']['velocity.ki']=int(self.settings.get('autotune.ki_result',0))
                self.autotune_log.append(dict(stage='probe',amplitude=amplitude,result='ok'))
                self.recorder.event('autotune_probe',amplitude=amplitude,result='ok',gains=dict(ctx['gains']))
                try:self._exchange('autotune apply')
                except Exception as exc:self._autotune_finish('Apply failed: '+str(exc));return
                self.note(f'Relay ID ok @ {amplitude} → applied; scoring baseline…')
                ctx['purpose']='baseline';self.autotune_phase='settling';self._autotune_deadline=now+2.0
                return
            self.autotune_log.append(dict(stage='probe',amplitude=amplitude,result=result))
            self.recorder.event('autotune_probe',amplitude=amplitude,result=result)
            ctx['amp_index']+=1
            if ctx['amp_index']>=len(AUTOTUNE_AMP_STEPS):
                self._autotune_finish('No oscillation up to the configured torque limit — check wiring/polarity');return
            amplitude=round(ctx['torqueClampMax']*AUTOTUNE_AMP_STEPS[ctx['amp_index']])
            try:self._exchange(f"autotune start relay {amplitude} {AUTOTUNE_PROBE_MAX_TICKS_STEPS[ctx['amp_index']]}")
            except Exception as exc:self._autotune_finish('Autotune start failed: '+str(exc));return
            self.note(f"{result} @ previous amplitude; trying {amplitude} ({ctx['amp_index']+1}/{len(AUTOTUNE_AMP_STEPS)})")
            return
        if self.autotune_phase=='settling':
            snap=self.model.snapshot()
            settled=bool(snap['latest']) and abs(snap.get('mean100ms_speed',0))<=1
            if settled or now>self._autotune_deadline:
                try:self._action('move',ctx['distance'],self._generation)
                except Exception as exc:self._autotune_finish('Validation move failed: '+str(exc));return
                self.autotune_phase='measuring';self._autotune_deadline=now+6.0
            return
        if self.autotune_phase=='measuring':
            snap=self.model.snapshot()
            move=snap.get('move')
            if snap['guard']:self._autotune_measurement_done(move,True,snap['guard']);return
            if move and move.get('settle_ms') is not None:self._autotune_measurement_done(move,False,None);return
            if now>self._autotune_deadline:self._autotune_measurement_done(move,True,'timeout');return

    def _autotune_measurement_done(self, move, failed, reason):
        # AUTOTUNE_TRIAL_REPEATS real moves per candidate, averaged, before any accept/reject
        # decision -- a single sample was repeatedly noisy enough on the bench (Ki landing
        # anywhere from 646-734, settle 425-720ms with otherwise-identical starting conditions)
        # to flip which candidate looked better.
        ctx=self._autotune_ctx
        ctx['repeat_costs'].append(self._trial_cost(move,failed));ctx['last_move']=move
        if len(ctx['repeat_costs'])<AUTOTUNE_TRIAL_REPEATS:
            self.autotune_phase='settling';self._autotune_deadline=time.monotonic()+2.0;return
        avg_cost=sum(ctx['repeat_costs'])/len(ctx['repeat_costs'])
        self._autotune_trial_done(ctx['last_move'],failed,reason,cost=avg_cost)

    def _autotune_trial_done(self, move, failed, reason, cost=None):
        ctx=self._autotune_ctx
        if cost is None:cost=self._trial_cost(move,failed)
        if ctx['purpose']=='baseline':
            ctx['best_cost']=cost;ctx['best_gains']=dict(ctx['gains']);ctx['best_move']=move
            self.autotune_log.append(dict(stage='baseline',gains=dict(ctx['gains']),cost=cost,failed=failed,reason=reason))
            self.recorder.event('autotune_baseline',gains=dict(ctx['gains']),cost=cost,failed=failed,reason=reason)
            self.note(f"Baseline cost {cost:.0f} — refining ({AUTOTUNE_REFINE_ROUNDS} rounds)…")
            ctx['coord_index']=0;ctx['round']=0;ctx['step_fraction']=AUTOTUNE_REFINE_STEP
            self._autotune_start_coord_trial(1);return
        coord=AUTOTUNE_REFINE_COORDS[ctx['coord_index']]
        improved=cost<ctx['best_cost']*(1.0-AUTOTUNE_MIN_IMPROVEMENT_FRACTION)   # must clear a margin, not just any improvement -- see AUTOTUNE_MIN_IMPROVEMENT_FRACTION
        self.autotune_log.append(dict(stage='refine',round=ctx['round'],coord=coord,trial=ctx['trial_kind'],cost=cost,kept=improved,failed=failed,reason=reason))
        self.recorder.event('autotune_refine',round=ctx['round'],coord=coord,trial=ctx['trial_kind'],cost=cost,kept=improved,failed=failed,reason=reason,gains=dict(ctx['gains']))
        if improved:
            ctx['best_cost']=cost;ctx['best_gains']=dict(ctx['gains']);ctx['best_move']=move
            self._autotune_advance_coord();return
        if ctx['trial_kind']=='up':
            self._autotune_start_coord_trial(-1);return
        # Neither up nor down improved on this coordinate -- restore it to its best known value
        # before moving on, so a rejected trial never stays live.
        try:self._exchange('set '+coord+' '+str(ctx['best_gains'][coord]))
        except Exception as exc:self._autotune_finish('Restore failed: '+str(exc));return
        ctx['gains'][coord]=ctx['best_gains'][coord]
        self._autotune_advance_coord()

    def _autotune_start_coord_trial(self, direction):
        # repeat_costs/purpose/trial_kind are reset BEFORE the blocking _exchange() below, not
        # after -- _exchange() releases the GIL during real serial I/O, and autotune_status()
        # runs on a different thread (the caller's poll loop), so clearing state only after the
        # exchange leaves a real window where it reads the PREVIOUS candidate's stale, already-
        # complete repeat_costs, producing a transient "(repeat 3/2)"-style display.
        ctx=self._autotune_ctx
        coord=AUTOTUNE_REFINE_COORDS[ctx['coord_index']]
        candidate=self._nudge(ctx['best_gains'][coord],ctx['step_fraction'],direction)
        ctx['trial_kind']='up' if direction>0 else 'down';ctx['purpose']='refine';ctx['repeat_costs']=[]
        try:self._exchange('set '+coord+' '+str(candidate))
        except Exception as exc:self._autotune_finish('Tuning step failed: '+str(exc));return
        ctx['gains'][coord]=candidate
        self.autotune_phase='settling';self._autotune_deadline=time.monotonic()+2.0

    def _autotune_advance_coord(self):
        ctx=self._autotune_ctx
        ctx['coord_index']+=1
        if ctx['coord_index']>=len(AUTOTUNE_REFINE_COORDS):
            ctx['coord_index']=0;ctx['round']+=1;ctx['step_fraction']/=2
        if ctx['round']>=AUTOTUNE_REFINE_ROUNDS:
            self._autotune_complete();return
        self._autotune_start_coord_trial(1)

    def _autotune_complete(self):
        ctx=self._autotune_ctx;move=ctx.get('best_move');gains=ctx['best_gains']
        label=self._verdict_label(move)
        detail=f"settle {move['settle_ms']:.0f} ms, overshoot {move['overshoot']} ct" if move and move.get('settle_ms') is not None else 'did not settle'
        amplitude=round(ctx['torqueClampMax']*AUTOTUNE_AMP_STEPS[ctx['amp_index']])
        self.recorder.event('autotune_complete',gains=gains,cost=ctx['best_cost'],move=move,amplitude=amplitude)
        self._autotune_finish(f"{label}: {detail} · Kp={gains['velocity.kp']} Ki={gains['velocity.ki']} P={gains['position.kp']} @ amp {amplitude}")

    def _autotune_finish(self, verdict):
        self.autotune_phase=None;self.autotune_verdict=verdict;self._autotune_ctx=None;self.note(verdict)
        self.recorder.event('autotune_finish',verdict=verdict,seq=self.model.last_seq)

    def _read_monitor(self):
        sync=FrameSync()
        while self.running.is_set():
            try:
                data=self._monitor.read(max(1,self._monitor.in_waiting))
                if not data:continue
                frames=sync.feed(data);self.recorder.write(data,frames)
                for f in frames:self.model.add(f)
            except Exception as exc:
                self.note('Monitor disconnected: '+str(exc));self.request('disable');return

    def _control(self):
        try:
            if self._cli:
                self._exchange('settings');self.saved_settings=dict(self.settings)
                self._exchange('monitor start')
                self.note('Connected; startup parameters pinned for Restore')
            while self.running.is_set():
                self._guard();self._autotune_tick()
                with self._condition:
                    if not self._queue:self._condition.wait(.03)
                    item=heapq.heappop(self._queue) if self._queue else None
                if not item:continue
                _,_,generation,action,value=item
                if generation!=self._generation:continue
                try:self._action(action,value,generation)
                except Exception as exc:
                    self.note('Command failed: '+str(exc))
                    # An incomplete enable/setup sequence must not leave motion active.
                    if self._cli:
                        try:self._exchange('disable');self._exchange('set velocity.command 0')
                        except Exception:pass
        except Exception as exc:
            self.note('Control connection failed: '+str(exc))
        finally:
            if self._cli:
                for command in ('autotune stop','disable','set velocity.command 0','monitor stop'):
                    try:self._exchange(command)
                    except Exception:pass
                self._cli.close()

    def _action(self, action, value, generation):
        if action=='quit':self.shutdown_requested=True;return
        if action=='hold':
            speed,seconds=value;seconds=float(seconds)
            if not 0<seconds<=60:raise ValueError('Hold duration must be 0..60 seconds')
            if int(speed)==0:raise ValueError('Select a nonzero velocity for a hold')
            self._action('velocity',int(speed),generation)
            if generation!=self._generation:return
            self.hold_deadline=time.monotonic()+seconds;self.hold_start_seq=self.model.last_seq
            self.recorder.event('hold_started',speed=int(speed),seconds=seconds,seq=self.hold_start_seq)
            self.note(f'Timed hold: {speed} counts/tick for {seconds:g} seconds');return
        if action=='mark':self.recorder.event('disturbance_marker',seq=self.model.last_seq,label=value or 'hand disturbance');self.note('Disturbance marked');return
        if action=='pin':
            snap=self.model.pin();self.recorder.event('baseline',snapshot=snap,seq=self.model.last_seq);self.note('Current score pinned as baseline');return
        if action=='autotune_start':
            if self.autotune_phase is not None:raise RuntimeError('Autotune already running')
            snap=self.model.snapshot()
            if snap['latest'] and snap['latest']['fault_latched']:raise RuntimeError('Fault latched; clear first')
            distance=int(value)
            if not 0<distance<=4096:raise ValueError('Validation distance must be 1..4096')
            self._exchange('settings')
            torqueClampMax=int(self.settings.get('torque.max',0))
            if torqueClampMax<=0:raise RuntimeError('Configure torque.max before autotuning')
            self.autotune_verdict=None;self.autotune_log=[]
            self._autotune_ctx=dict(distance=distance,torqueClampMax=torqueClampMax,amp_index=0,
                gains={k:0 for k in AUTOTUNE_REFINE_COORDS},best_gains=None,best_cost=None,best_move=None,
                purpose=None,coord_index=0,trial_kind='up',round=0,step_fraction=AUTOTUNE_REFINE_STEP,
                repeat_costs=[],last_move=None)
            amplitude=round(torqueClampMax*AUTOTUNE_AMP_STEPS[0])
            self._exchange(f'autotune start relay {amplitude} {AUTOTUNE_PROBE_MAX_TICKS_STEPS[0]}')
            self.autotune_phase='probing';self._autotune_poll_at=0
            self.recorder.event('autotune_start',amplitude=amplitude,distance=distance,seq=self.model.last_seq)
            self.note(f'Autotune: probing amplitude {amplitude} (1/{len(AUTOTUNE_AMP_STEPS)})');return
        if action=='autotune_stop':
            try:self._exchange('autotune stop')
            except Exception:pass
            if self.autotune_phase is not None:self.autotune_verdict='Cancelled'
            self.autotune_phase=None;self._autotune_ctx=None;self.note('Autotune cancelled');return
        if action in ('disable','stop'):
            if self.autotune_phase is not None:
                try:self._exchange('autotune stop')
                except Exception:pass
                self.autotune_phase=None;self._autotune_ctx=None;self.autotune_verdict='Cancelled'
            was_point=self.model.move is not None
            self.model.cancel_move()
            self.model.cancel_move()
            self.requested_speed=0
            commands=['disable','set velocity.command 0','set input.source host'] if action=='disable' else (['set input.source host','set velocity.command 0'] if was_point else ['set velocity.command 0'])
            self._sequence(commands,generation);self.note('Disabled' if action=='disable' else 'Ramping to zero');return
        if action=='refresh':self._exchange('settings');return
        if action=='clear':
            self._sequence(['disable','set velocity.command 0','set input.source host','fault clear','settings'],generation)
            self._guarded=self._fault_seen=False;self.note('Firmware fault cleared; check amplifier separately');return
        if action=='restore':
            was_point=self.model.move is not None
            self.model.cancel_move()
            self.model.cancel_move()
            self.requested_speed=0
            commands=['disable','set velocity.command 0','set input.source host']
            allowed=lambda k:k in ('position.kp','velocity.kp','velocity.ki','trajectory.accel_limit','trajectory.jerk_limit','torque.accel_feedforward','trajectory.move_feedforward','velocity.filter_shift','torque.min','torque.max','velocity.limit','trajectory.velocity_limit','output.trim','position.error_limit','position.error_trip','torque.sustained_threshold','torque.sustained_ticks')
            commands += ['set '+k+' '+v for k,v in self.saved_settings.items() if allowed(k)]
            self._sequence(commands+['settings'],generation);self.note('Startup profile restored in RAM; disabled');return
        if action=='save':
            snap=self.model.snapshot()
            if not snap['latest'] or snap['latest']['enabled'] or abs(snap['latest']['velocity'])>0:
                raise RuntimeError('Disable and stop before saving a startup profile')
            self._sequence(['set velocity.command 0','set input.source host','config save','settings'],generation)
            self.saved_settings=dict(self.settings)
            (self.recorder.directory/'saved-settings.json').write_text(json.dumps(self.saved_settings,indent=2)+'\n')
            self.note('Saved disabled configuration to flash');return
        if action=='gain':
            key,number=value
            if key not in ('position.kp','velocity.kp','velocity.ki','trajectory.accel_limit','trajectory.jerk_limit','torque.accel_feedforward','trajectory.move_feedforward','velocity.filter_shift'):raise ValueError('Unsupported tuning parameter')
            number=int(number)
            self._sequence(['set '+key+' '+str(number),'settings'],generation);self.note(key+' = '+str(number));return
        if action in ('velocity','move'):
            if not self._settings_ready:raise RuntimeError('Waiting for device settings')
            if self.settings.get('output.polarity_learned')!='1':raise RuntimeError('Motor polarity must be commissioned first')
            snap=self.model.snapshot()
            if snap['stale'] or not snap['latest']:raise RuntimeError('Fresh telemetry required')
            if snap['latest']['fault_latched']:raise RuntimeError('Fault latched; inspect drive, then clear explicitly')
            self._guarded=False
            if action=='velocity':
                self.hold_deadline=None
                self.model.cancel_move()
                speed=int(value)
                if abs(speed)>self.max_command:raise ValueError('Outside operator command envelope')
                limit=max(abs(speed),int(self.saved_settings.get('trajectory.velocity_limit','40')))
                correction=max(int(self.saved_settings.get('velocity.limit','60')),math.ceil(limit*1.5))
                commands=[]
                if self.settings.get('trajectory.velocity_limit')!=str(limit):commands.append('set trajectory.velocity_limit '+str(limit))
                if self.settings.get('velocity.limit')!=str(correction):commands.append('set velocity.limit '+str(correction))
                if not snap['latest']['enabled']:commands+=['set input.source host','set velocity.command 0','enable']
                commands+=['set velocity.command '+str(speed)]
                self.requested_speed=speed
                self._sequence(commands,generation);self.note('Velocity '+str(speed)+' counts/tick')
            else:
                if abs(snap['mean100ms_speed'])>self.move_settle_threshold:raise RuntimeError('Stop velocity before starting a point move')
                distance=int(value)
                if abs(distance)>4096:raise ValueError('Move exceeds 4096-count bench envelope')
                target=snap['latest']['position']+distance
                commands=['set input.source host','set velocity.command 0']
                if not snap['latest']['enabled']:commands+=['enable']
                self._sequence(commands+['set position.command '+str(target)],generation)
                self.model.begin_move(target,snap['latest']['position'],self.last_command_seq)
                self.recorder.event('point_move',target=target,distance=distance,seq=self.last_command_seq)
                self.note('Move '+str(distance)+' counts')
            return
        raise ValueError('Unknown action '+str(action))

    def close(self):
        if self.control_socket:self.control_socket.close()
        self.request('disable')
        # Drain the priority disable before closing either port.
        time.sleep(.4)
        self.running.clear()
        with self._condition:self._condition.notify_all()
        for t in self._threads:t.join(timeout=3)
        if self._monitor:self._monitor.close()
        self.recorder.close(dict(score=self.model.snapshot(),settings=self.settings,baseline=self.model.baseline,messages=self.messages))

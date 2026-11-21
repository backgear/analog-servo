# Control-loop plots with simmy

**Synthetic simulation, not bench-validated.** These plots exercise the real
`firmware/src/Control.c`, not the whole STM32 application, interrupts, encoder
hardware, fault logic, or electrical pulse timing. Model coefficients and gains
are representative, not measurements or recommended hardware tuning.

## Run

Requires a host C compiler, Python 3 with matplotlib, and a simmy source checkout.
No changes or builds inside the simmy checkout are necessary.

```sh
python3 -m pip install matplotlib  # preferably in your plotting virtualenv
python3 tools/simmy_plot.py --simmy-dir ~/simmy
# Or use the optional PlatformIO native custom target:
SIMMY_DIR=~/simmy pio run -d firmware -e native -t simmy
```

`SIMMY_DIR` defaults to `~/simmy`. `SIMMY_PYTHON=/path/to/venv/bin/python`
selects the plotting interpreter for the PlatformIO target. Direct execution
uses the Python you invoke. `CC` selects the host compiler (default `cc`).
The board build and `pio test -d firmware -e native` do not require simmy.

Default outputs (ignored by git):

- `.cache/simmy/frames.json`: telemetry-shaped JSON for the existing renderer.
- `.cache/simmy/telemetry.png`: position, velocity, torque panels. The position
  panel includes signed following error (`cmd_position - position`) on the
  purple right axis; positive means actual position trails a positive move.
  Unlike the displayed position, error is not rebased at the window start.
- `.cache/simmy/run.json`: parameters, plant configuration, firmware/simmy git
  descriptions (including dirty markers), tick rate and torque limits. For
  reproducible published comparisons use clean, pinned commits; a dirty marker
  alone cannot reconstruct local edits.

Each invocation recompiles the current controller and simmy sources. Save
before/after results in separate directories:

```sh
python3 tools/simmy_plot.py --out-dir .cache/before
# Edit Control.c, then:
python3 tools/simmy_plot.py --out-dir .cache/after
# Alternative plant, or tune the raw Q8.8 gains:
python3 tools/simmy_plot.py --config /path/to/plant.json --pos-kp 32 --vel-kp 1024 --vel-ki 1
# Original conservative gains for comparison:
python3 tools/simmy_plot.py --tune baseline --out-dir .cache/simmy-baseline
# No matplotlib dependency if you only need data:
python3 tools/simmy_plot.py --no-plot
python3 -m unittest discover -s tools -p test_simmy_plot.py
```

The default plant lives here at `tools/simmy/two_mass.json`, copied from simmy's
MIT-licensed `examples/configs/two_mass.json`. Keeping the scenario in this repo
avoids silently changing the experiment when an upstream example changes.
See simmy's README/header for the JSON schema and its modeling limitations.
Different models require different gains; no stability claim is made for
arbitrary combinations.

## One-command nine-case report

```sh
python3 tools/simmy_report.py
# Equivalent PlatformIO custom target:
pio run -d firmware -e native -t simmy-report
# Data + Markdown only (does not require matplotlib):
python3 tools/simmy_report.py --no-plots --out-dir .cache/simmy-data-report
```

Default output: **`.cache/simmy-report/report.md`**. It compares baseline,
tracking-focused (`optimized`), and balanced tunes, each with no feedforward,
velocity feedforward, and velocity+acceleration feedforward: nine independent
runs of the full 10-second sequence. The driver is compiled once per batch.
`--scenario ramp-cruise` runs the original two-second experiment instead;
`--rate`, `--ramp-seconds`, `--jitter`, `--ff-shift` and `--accel-ff-q8` control
shared inputs. All nine cases see identical target positions and pulse jitter.

Artifacts include:

- `tunes-none.png`, `tunes-velocity.png`, `tunes-velocity-acceleration.png`:
  one column per tune, common primary and error-axis limits across columns.
- `balanced-feedforward.png`: one column per feedforward option, balanced gains.
- Matching comparison JSON files for re-rendering.
- `scores.json`: full-run, motion-only, per-phase and hold metrics for every case.
- `manifest.json`: batch arguments, revisions and report-source hash.
- `<tune>-<feedforward>/frames.json`, `metrics.json`, `run.json`: individual
  experiment data with resolved gains, fixed seeds, plant content and SHA-256
  hashes of C/header inputs and plotting/metric sources. Dirty external simmy
  sources are not modified; hashes detect their differences but are not source
  snapshots, so archive or commit those inputs to reproduce a published run.

Do not choose gains only from whole-run RMS: the holds can expose oscillation
that moving-only error obscures. On the default sequence, gains-only results
are:

| Metric | Baseline | Tracking | Balanced |
| --- | ---: | ---: | ---: |
| Motion-only motor RMS | 85.6 | 7.0 | 10.1 |
| Motion-only load RMS | 96.5 | 17.3 | 19.2 |
| Whole-run torque RMS | 55.5 | 144.3 | 61.3 |
| Whole-run torque-increment RMS | 2.3 | 66.2 | 15.0 |

None of these three establishes both motor and load settling in either hold.
The tracking-focused preset is particularly active at standstill. The report
records this rather than declaring an overall winner. Fixing it requires a
separate stop/low-speed control experiment, not concealing it with plot filters.

Tests: `python3 -m unittest discover -s tools -p 'test_simmy*.py'` and
`python3 -m unittest discover -s tools -p test_plot_telemetry.py`.
External-checkout integration tests skip explicitly if simmy is unavailable;
metric and renderer tests still run independently (renderer needs matplotlib).

## Stopping investigation (experimental, partial improvement)

```sh
python3 tools/simmy_stop_report.py
# Data/report only:
python3 tools/simmy_stop_report.py --no-plots
# Individual candidate run:
python3 tools/simmy_plot.py --scenario sequence --tune balanced --feedback encoder \
  --idle-gains 64 2048 16 0 1 --out-dir .cache/simmy-idle
```

Read **`.cache/simmy-stopping-report/report.md`**, `forward-stop.png` and
`final-stop.png`. This 23-case report retains failures as well as successes:
model/encoder feedback, deadband-only, an idle schedule, constant low-bandwidth
gains, ripple/friction ablations, more mechanical substeps, other speeds and
directions, sparse pulses, zero jitter and an extended final hold.

### Sensor fidelity

`--feedback encoder` uses current minus previous integer position, matching
Encoder.c's measurement convention; this also applies to the measured load
velocity. The initial previous position is the initial encoder count.
`--feedback model` (the single-run default for comparability with the tune
experiments above) rounds instantaneous model velocity instead. These differ
near encoder transitions and during acceleration. Metadata records the mode.
Use encoder mode for new firmware-facing stopping investigations; neither mode
models real timer/electrical timing or wraparound. Changing measurement mode
alone does not remove the limit cycle.

### Pulse-idle gain candidate

`--idle-gains POS_KP VEL_KP VEL_KI FILTER_SHIFT DEADBAND` is a **host-only,
opt-in gain schedule**, not a new controller implementation or firmware default.
After `--idle-after-ticks 50` consecutive zero pulse increments, it reapplies
existing ControlGains through ControlSetGains. Any new pulse restores the moving
gains before that tick's control call. All PI state is retained: no reset,
position snap, torque-off, integral leak, analytic phase lookup or load input.
The threshold uses only counted pulse history, so very slow pulse trains can
also trigger it; inactivity is not proof of a commanded stop. `idle_active`
records the decision every tick. Explicit gain/deadband values remain in run
metadata. The candidate uses Q8.8 **64 / 2048 / 16**, filter 0, existing velocity
error deadband 1. The latter is not an unconditional position-error deadband;
large errors still reach the controller.

On the default unchanged plant, with encoder-delta feedback:

| Hold measurement | Balanced, unchanged | Pulse-idle candidate |
| --- | ---: | ---: |
| Forward motor settling time | Not settled | 0.077 s |
| Final motor settling time | Not settled | 0.076 s |
| Forward final-250-ms motor span | 48 counts | 0 counts |
| Final final-250-ms motor span | 57 counts | 0 counts |
| Forward tail torque ripple RMS (DC removed) | 62.7 | 0.0 |
| Final tail torque ripple RMS (DC removed) | 64.3 | 0.0 |
| Motion-only motor error RMS | 11.0 counts | 10.7 counts |
| Motion-only load error RMS | 19.6 counts | 19.9 counts |

This is a **partial improvement, not a robust stopping fix**. At 1000 counts/s,
the candidate's final motor error remains 6 counts and fails the unchanged
±5-count settling criterion. Disabling friction exposes severe moving-loop
instability even if the idle schedule recovers at a later hold. More substeps
and removing ripple alone do not cure the unchanged controller's hold cycle.
The moving tune's reliance on plant friction/damping needs a separate stability
review. Do not select a preset from one favorable endpoint or loosen the
metrics to declare success. Existing deadband alone does not solve this case.

### Motor settling is not load centering

The candidate still leaves the forward load 11 counts off target, despite a
stationary motor within 2 counts. For the bundled plant, static load friction
80 and spring stiffness 3.16 plus backlash half-gap 0.5 can support approximately
`80/3.16 + 0.5 = 25.8` counts of static deflection. The motor encoder cannot tell
which admissible load offset is present. Motor-only control cannot guarantee
±5-count load positioning in that model: mechanics, load measurement, or the
physical positioning requirement must be revisited separately. This is not a
reason to feed simulator load truth back into the controller.

Hold metrics now include final-250-sample motor/load position span, velocity
RMS, mean torque and DC-removed torque-ripple RMS. These distinguish being in
the tolerance band from a quiet hold, and steady holding effort from chatter.
None of these host experiments is bench-validated or enabled in production.

## C → Python boundary

```text
simmy pulse source → target position
                           ↓
quantized feedback → real ControlTick() → integer torque command
       ↑                                     ↓
  simmy mechanics ← simmy analog chain (DAC/amplifier stand-in)
                           ↓
                 JSON frames on stdout
                           ↓
            Python subprocess + json.loads
                           ↓
                tools/plot_telemetry.py
```

`firmware/test/support/simmy_plot_main.c` owns the C loop. It links `Control.c`
and simmy's three `.c` files directly, avoiding stale prebuilt archives. A
`SimmyControllerFn` adapter defaults to `ControlTick()` with no feedforward,
matching the current Mode A entry path. Opt-in feedforward experiments use
`ControlTickWithReferenceQ8()` instead. This does **not** run Mode B/C's trajectory
pipeline; changes to Ramp.c, PositionMove.c, or main.c are not exercised here.

The clock is fixed at 1 kHz (32000 CPU cycles at 32 MHz); PI gains are per tick.
Gains are raw Q8.8 integers (`256` = `1.0`). Torque is a separate signed integer
scale, not Q8.8 or N·m; this scenario limits it to ±1000 within the firmware's
±32767 full scale. Physical conversion, if desired, belongs in the analog
chain's gain stage and calibrated plant coefficients.

Feedback is sampled before plant advancement. `cmd_velocity` is the cascade's
velocity target, not pulse-source velocity. `fault_latched=false` is only a
renderer compatibility field: no fault detector runs in this harness.
`load_position`/`load_velocity` record the last body's quantized motion. When
present on every frame, the renderer adds a separate load-position panel with
signed load error on its right axis, and a load trace on the velocity panel.
Both position panels use the feedback body's initial position as their common
origin, so load offsets are not hidden. The first panel's “actual” trace is the
configured feedback body; captures without load fields keep three panels. Seeds are fixed for
repeatable pulse jitter and analog noise. Negative `--rate` is supported;
the default `ramp-cruise` scenario ramps once then cruises. Use
`--scenario sequence` for a complete move/stop/reverse experiment.

A subprocess + JSON is deliberately the first integration: no Python C ABI,
shared-library lifecycle, or per-tick Python overhead. Consider ctypes/cffi or
pybind11 later only if interactive Python needs to step the plant or replace
controllers in-process. CSV is another suitable batch boundary, but JSON
already fits this repo's telemetry plotter unchanged.

## Complete move sequence and settling

```sh
python3 tools/simmy_plot.py --scenario sequence --tune balanced --out-dir .cache/simmy-sequence
```

Without `--ticks`, this runs the entire sequence (10 seconds at the default
one-second ramp). Each direction has acceleration, a one-second cruise,
deceleration, and a two-second hold. The state of the pulse source, plant and
controller is continuous: there is no integrator reset or artificial position
snap at either stop or reversal. Ramp durations are rounded to integer 1 ms
ticks. The default boundaries are 0/1/2/3/5/6/7/8/10 seconds. `phase` and
`target_rate` are saved on each frame; jitter means return position need not
be exactly zero. Explicit `--ticks` may truncate or extend the experiment.

Every run also writes `metrics.json`, including motor/load RMS and peak error,
torque RMS, torque-increment RMS, exact clamp-hit counts and hold measurements.
Settling requires absolute error ≤5 counts AND absolute velocity ≤1 count/tick
for all remaining samples in the recorded hold, with at least 250 ms of such
evidence. `null` means not settled in that observation window, not success.
These thresholds are explicit evaluation choices, not hardware specifications.
The balanced preset's two default holds do not meet this criterion: residual
standstill oscillation remains despite much better tracking during motion.

## Simulation-specific optimized tune

The runner defaults to `--tune optimized`: raw Q8.8 gains **256 / 3072 / 32**
(position P / velocity P / velocity I), equivalent to 1.0 / 12.0 / 0.125.
Filtering remains off; neither feedforward nor any plant parameter changed.
`--tune baseline` selects the original **32 / 1024 / 1**. Individual gain flags
still override the selected preset. Neither preset changes firmware defaults,
and neither is a recommendation for real hardware or arbitrary custom plants.

A finite grid search compared 240 coarse combinations, followed by 128 local
combinations. The coarse grid used position P {32,64,128,256,512,1024}, velocity
P {1024,2048,4096,8192}, velocity I {0,1,4,16,64}, filter shift {0,1}; the local
grid used position P {192,256,320,384}, velocity P {1536,2048,2560,3072}, velocity
I {16,32,64,128}, filter shift {0,1}. The score was motor error RMS + load error
RMS + 0.05 × torque-increment RMS + 0.01 × torque RMS + number of ticks at
|torque| ≥999, over the full default two-second simulation. This is a chosen
tracking/activity compromise, not a globally optimal tune or a physical energy
metric. Shortlisted candidates were then checked at longer durations and other
rates, rather than selecting purely by the appearance of the motor trace.

| Full two-second scenario | Baseline | Optimized |
| --- | ---: | ---: |
| Feedback-body following-error RMS (counts) | 92.0 | 6.0 |
| Feedback-body peak absolute error (counts) | 273 | 19 |
| Load following-error RMS (counts) | 104.9 | 18.4 |
| Load peak absolute error (counts) | 318 | 63 |
| Torque RMS (command units) | 68.6 | 91.2 |
| Tick-to-tick torque increment RMS | 3.0 | 34.0 |
| Ticks at absolute torque ≥999 | 0 | 0 |

The improvement costs increased torque activity; no filtering is applied to
hide that in the graph. Load tracking is still worse than motor tracking due
to the compliant/frictional plant. Eight-second runs at +2500, -2500, 1000 and
5000 counts/s had motor peak errors ≤27 counts and load peak errors ≤65 counts,
with no clamp hits. The separate full-sequence experiment covers stopping and reversal after a
hold, but these checks do not cover rapid reversal, parameter uncertainty,
noise robustness or real hardware stability. Integration
tests retain the baseline case and guard motor/load error for the tuned cases.

## Balanced, lower-chatter preset

Use `--tune balanced` for raw Q8.8 **256 / 1024 / 16**, measured-velocity filter
shift **1** (alpha 1/2). It leaves the plant and input disturbances unchanged.
This sacrifices some motor tracking precision to reduce torque activity:

| Full two-second scenario | Tracking (`optimized`) | `balanced` |
| --- | ---: | ---: |
| Motor error RMS (counts) | 6.0 | 9.6 |
| Load error RMS (counts) | 18.4 | 19.9 |
| Torque RMS | 91.2 | 63.9 |
| Torque-increment RMS | 34.0 | 11.7 |

Reproduce the 144-candidate local search with `python3 tools/simmy_tune.py`.
It varies position P {192,256,320}, velocity P {512,768,1024,1280}, velocity I
{8,16,24,32}, filter shift {0,1,2}, scoring motor RMS + load RMS + 0.8 × torque
increment RMS + 0.1 × torque RMS + saturation ticks. Coefficients encode an
explicit preference, not physical energy. Eight-second ramp/cruise checks at
+2500, -2500, 1000, 5000 counts/s showed motor peak error ≤48, load peak ≤83,
and no clamp hits. A lower score on this search does not prove robustness.

## Why the baseline trace is jumpy

The default plant is a disturbance/stick-slip stress case, not a smooth reference
plant. It combines a compliant coupling, load static/Coulomb friction (80/50),
motor ripple (amplitude 60, period 20 counts), and an extra localized friction
patch near load position 1800. The position error and velocity PI integral can
build torque against a stuck load, storing energy in the coupling; breakaway
then produces a burst of motion. Ripple and the friction patch add further
excursions. Whole-count velocity feedback at a mean 2.5 counts/tick adds visible
quantization, but is not the whole explanation.

One-at-a-time synthetic comparisons with `--tune baseline`, over the cruise
interval 1.000–1.999 seconds (RMS of signed feedback-body following error):

| Variant | Error RMS (counts) |
| --- | ---: |
| Default | 86.4 |
| No pulse jitter | 87.6 |
| Velocity filter shift 1 | 82.8 |
| Velocity filter shift 3 | 78.9 |
| No ripple | 67.8 |
| No localized friction bump | 64.3 |
| No static/Coulomb friction, including the bump | 54.5 |
| No static/Coulomb friction, bump, ripple, or jitter | 19.3 |

These are short, model-specific comparisons, not hardware tuning evidence.
For the variants, set the corresponding body fields in a copy of the config
(`stictionLimit`, `coulombLevel`, `frictionBumpExtra`, `rippleAmplitude`) to zero;
use `--tune baseline` and `--jitter 0` or `--filter-shift N` as appropriate.

Filtering measured velocity does not remove actual stick-slip and introduces
feedback delay; it is not the first fix for this trace. Velocity feedforward
can reduce the baseline tracking lag: at 2500 counts/s and `posGainQ8=32`, a
pure position P loop needs about `(2500/1000)/(32/256) = 20` counts of error to
request the cruise speed. It does not by itself compensate static friction or
ripple. Acceleration feedforward is a separate, model-dependent torque term
for acceleration, not a general cure for cruise-speed disturbances. The default run uses neither feedforward path; both are opt-in experiments.

For a smooth baseline, disable the deliberate disturbances first, then add
ripple, friction, compliance and jitter back separately. Tune against motor
**and load** motion, not just the motor encoder. Treat any proposed feedforward,
friction compensation or revised gains as a separate experiment; do not hide
real tracking error by smoothing the plotted position/error traces.

## Experimental feedforward comparisons

```sh
python3 tools/simmy_plot.py --scenario sequence --tune balanced --feedforward velocity
python3 tools/simmy_plot.py --scenario sequence --tune balanced --feedforward velocity-acceleration
```

These are **EXPERIMENTAL Mode A paths, not current production Mode A behavior**.
The reference is generated solely from this tick's quantized pulse increment:
`referenceQ8 += (delta*256 - referenceQ8) / (1 << ff_shift)` with C integer
truncation. `--ff-shift 3` is default; 0 exposes raw counts. The estimator starts
at zero and retains its state through stops and reversals. It does not read
analytic `target_rate`, future position samples, motor/load velocity or plant
parameters. Frames expose `pulse_delta` and `reference_velocity_q8` so this
causality can be checked. Velocity feedforward changes the speed reference,
not a direct friction-compensating torque command.

The separate `velocity-acceleration` mode enables Control.c's existing causal
acceleration-torque path with `--accel-ff-q8 18432` (72.0 raw command units per
count/tick², based on the model's combined 64+8 inertias, not a fitted hardware
coefficient). In other modes its effective gain is zero. This exploratory
coefficient neglects compliance, amplifier lag and friction; acceleration
feedforward is **not optimized** here. Control.c's existing anti-windup/clamp
still applies. Set its gain to zero to reproduce velocity-only output exactly.

For the balanced tune on the full default sequence:

| Feedforward | Motor RMS | Load RMS | Torque RMS | Torque-increment RMS |
| --- | ---: | ---: | ---: | ---: |
| None | 11.2 | 15.6 | 61.3 | 15.0 |
| Velocity | 11.4 | 15.3 | 62.1 | 15.0 |
| Velocity + acceleration | 11.4 | 15.0 | 62.0 | 15.5 |

None meets both bodies' settling criterion during the holds. Feedforward is
not a clear general improvement on this strong-friction/ripple model at these
gains; keep the no-feedforward case as the reference. Changes to firmware
behavior require independent review and bench validation.

## External repositories and PlatformIO: no submodule required

PlatformIO can consume external code without the repo being a PlatformIO
*project*. The choice depends on whether it needs to build the code as a
*library*, explicitly add source files, or just invoke a host tool.

| Option | Appropriate use | Tradeoff |
| --- | --- | --- |
| `lib_deps = https://host/owner/repo.git#<commit>` | Pinned remote C/C++ library for CI and collaborators | Must have a layout PlatformIO understands; add a `library.json` upstream for explicit include/source filters and platform compatibility. Network access on first install. |
| `lib_deps = symlink:///absolute/path/to/repo` | Live local library development, edits visible immediately | Machine-specific path; layout still must be recognized as a library. Not a reproducibility mechanism. |
| `lib_deps = file:///absolute/path/to/repo` | Installing a local library snapshot | Copied/installed dependency; do not assume every source edit is reflected as with a symlink. |
| `extra_scripts` + SCons `BuildSources`/include paths | Nonstandard source layout or exact source selection | You own host/MCU separation, compiler flags and dependency tracking. Avoid adding all external `.c` files blindly. |
| Custom target invoking a host script (**used here**) | Offline simulation and chart generation | Separate host compiler and Python dependencies, explicitly invoked. No need for PlatformIO library metadata. |
| Git submodule + one of the above | Pinning a checkout inside this repo | Git handles checkout/versioning, **not compilation**. Collaborators/CI must initialize submodules; local edits and updates need extra care. |
| Vendoring sources | Self-contained snapshot | Updates and license preservation are your responsibility. |

`-I/path/include` alone only exposes headers; it does not compile/link external
implementations. `lib_extra_dirs` is an older library-search mechanism, not a
necessary solution here; prefer explicit dependencies or a build script.

For this workflow, start with the local checkout and this explicit host target.
Once sharing/CI needs it, pin a simmy commit via a CI checkout or a submodule
(e.g. `external/simmy`) and set `SIMMY_DIR` accordingly. If simmy later adds a
PlatformIO library manifest, a pinned Git `lib_deps` in a **separate native
simulation environment** is also reasonable. Do not place it in shared `[env]`
or the STM32 board's dependencies: the mechanical simulator and JSON file I/O
belong on the host, not in the MCU image.

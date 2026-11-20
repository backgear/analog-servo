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
`SimmyControllerFn` adapter calls `ControlTick()` with no feedforward, matching
the current Mode A entry path. This does **not** run Mode B/C's trajectory
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
the initial runner ramps once then cruises, not a forward/reverse sequence.

A subprocess + JSON is deliberately the first integration: no Python C ABI,
shared-library lifecycle, or per-tick Python overhead. Consider ctypes/cffi or
pybind11 later only if interactive Python needs to step the plant or replace
controllers in-process. CSV is another suitable batch boundary, but JSON
already fits this repo's telemetry plotter unchanged.

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
with no clamp hits. These checks do not cover stopping, rapid reversal,
parameter uncertainty, noise robustness or real hardware stability. Integration
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
for acceleration, not a general cure for cruise-speed disturbances. This
runner currently uses neither feedforward path.

For a smooth baseline, disable the deliberate disturbances first, then add
ripple, friction, compliance and jitter back separately. Tune against motor
**and load** motion, not just the motor encoder. Treat any proposed feedforward,
friction compensation or revised gains as a separate experiment; do not hide
real tracking error by smoothing the plotted position/error traces.

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

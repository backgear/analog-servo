# Two-phase commutation output (`output.topology=two_phase_analog`)

`two-phase-output` draft-PR branch, not in `main`. **Not bench-verified** --
needs a real Hall sensor and the CD4053 sign-switch hardware described
below, neither on hand yet. `main`'s single-channel `analog_torque` output
is unaffected: this branch adds `env:analog_servo_two_phase` alongside
`env:analog_servo`, gated by `ANALOG_SERVO_TWO_PHASE_DRIVE` so the default
build never compiles `Hall.c`/`Commutation.c` in or pays for this code.

## What this is, and isn't

For a drive that expects **two** analog phase-torque commands instead of
today's single scalar one (and infers the third phase itself, since a
balanced three-phase system sums to zero). This firmware still commands an
*external* drive exactly as `main` does -- it does not switch phases or
close a current loop itself. Target: AMC's SX25A20A-GF1 (an AMC "S/SX"
family drive -- *"Two sinusoidal command signals (120 degrees out of
phase) from the controller control the commutation and torque of the
motor... All feedback goes to the controller, not the drive, including
commutation feedback"* -- per AMC's generic *Hardware Installation Manual*,
since this exact variant has no public datasheet). No gate drivers, no
phase switching, no current sensing get added here -- the external drive
keeps doing all of that; `docs/limitations.md`'s "we don't measure real
current" caveat is unaffected.

## Why two channels isn't enough, and the scheme this branch implements

Each phase needs its own two unipolar channels, same reason `main`'s single
phase does (recovering a bipolar signal from the L152's unipolar DAC
channels). Two phases x two channels = **4 analog channels needed; `main`'s
architecture has exactly 2, already fully spent on the one existing
phase.** The L152 has exactly one DAC peripheral with exactly 2 channels
(`PA4`/`PA5`) -- zero spare DAC channels, no second DAC peripheral to reach
for. This needs new output hardware, not a firmware-only repurposing.

**The scheme implemented: unsigned magnitude per channel + a separate sign
GPIO per phase, no analog reference at all.** Each channel keeps the
*full* `0->max` code range as pure magnitude (0 = zero command) instead of
splitting it across both directions -- full per-channel resolution, no
reference voltage anywhere in the path to drift. Sign is carried
separately, one GPIO per phase (`gpio.h`'s `PHASE_A_SIGN_PIN`/
`PHASE_B_SIGN_PIN`), each driving an external analog switch (a CD4053: 2 of
its 3 sections as a shared-select DPDT, cross-wired so one switch state
presents `(magnitude signal, GND)` on the drive's `+`/`-` terminals and the
other presents `(GND, magnitude signal)`). This is strictly cheaper and
safer than an earlier VCC/2-reference alternative that was tried and
superseded: that scheme halved per-channel resolution and made "true zero"
depend on how precisely an external reference held VCC/2 over
time/temperature -- a worse failure mode (nonzero commanded torque at
rest) than losing a bit of resolution. The sign-switch scheme has no such
reference anywhere in the signal path.

`Output.c`'s `OutputSetTwoPhaseCommand()`/`OutputSplitTwoPhase()` implement
this, reusing the same two DAC channels `OutputSetTorqueCommand()` already
uses -- `OutputSetTopology()` tells `OutputForceSafe()` which scheme's
"zero" to write; `main.c` still picks which `Output*Command()` function to
call each tick based on `output.topology`. **The hardware side (the actual
CD4053 sign-switch circuit) is not built on any board** -- firmware written
ahead of the hardware it assumes. Until that hardware exists and is
bench-measured, treat this entire branch as unverified.

## Hall input pins

`HALL_A`/`HALL_B`/`HALL_C` -- `PC0`/`PC1`/`PC2` on the L152, pull-up inputs,
**polled, not EXTI** (`main.c`'s `AppControlTick()`, once per 1kHz tick).

**Why polling, not EXTI:** Hall edge rate is `RPM x pole_pairs / 10`
edges/sec. Checked against a 1ms poll period, a 4-pole motor only crosses
the "might miss an edge" line above roughly 5000-6000 RPM -- not a
dramatic concern for most real motors at realistic speeds. More
importantly, **a missed Hall edge isn't the same class of problem as a
missed encoder/index edge.** Those are edge-counted/relative -- miss one
and the running count is permanently wrong. Three Hall lines instead
encode an absolute level (which of 6 valid 3-bit states is present *right
now*) -- a 1kHz poll is self-correcting: the next tick's read is still the
true current sector regardless of how many transitions happened in
between. The real cost of polling is commutation lag, up to one tick's
staleness at the instant a sector changes -- a torque-ripple/smoothness
cost, not a correctness failure, and this only matters during the
pre-Z-index window at cold boot anyway (see below) -- a startup/search
phase where low RPM is the realistic case. At 200 RPM on a 4-pole motor, a
full 1ms poll lag is only ~2.4 degrees of angle error (~0.04% efficiency
loss). EXTI would also cost real code (3 separate vectors, 3
`HAL_NVIC_SetPriority`/`EnableIRQ` pairs, a new callback-switch case) that
polling needs none of. So: poll it. EXTI stays a legitimate later upgrade
if real bench data ever shows otherwise.

**Scoped correctly, this only matters for part of the feature's
lifetime.** The *continuous* angle once running is `pole_pairs x
mechanical_angle` off the encoder's own hardware counter -- a live
register read with no polling-rate-dependent staleness at all,
Hall-independent. Hall only actually drives real-time torque production
during the pre-Z-index window at each cold boot, before the encoder's
absolute reference has been established for this power cycle -- exactly
the gap Hall was chosen to fill. After Z is seen, Hall's only remaining job
is the one-time alignment reference.

## Commutation-angle stage (`Commutation.c`)

Sits between `Control.c`'s velocity(PI) output and the two-phase output:
takes the scalar torque magnitude+sign and the current electrical angle
and produces `(Ia, Ib)` (`CommutationComputePhases()`, a small fixed-point
sine table, no floating point). The angle itself is Hall-sourced
pre-Z-index (`CommutationElectricalAngleFromHallDeg()`, 60-degree steps)
and encoder-sourced once Z has been seen
(`CommutationElectricalAngleFromEncoderDeg()`, continuous,
offset-corrected).

## Pole-count / counts-per-electrical-rev detection

`commutation.pole_pairs`/`commutation.counts_per_elec_rev` are pure manual
guesses (`ConfigStore.h`'s `CONFIG_DEFAULT_COMMUTATION_*`) until
`commutation detect_poles <encoder_cpr>` actually runs
(`main.c`'s `AppCommutationDetectPoles`, `Commutation.h`'s
`CommutationPoleLearnBySpacing*`): averages encoder counts between
consecutive Hall-sector transitions (60 electrical degrees each);
`counts_per_elec_rev` = 6x that average, `pole_pairs` = `encoder_cpr` /
`counts_per_elec_rev`. Requires the caller to already know the encoder's
counts-per-mechanical-revolution (`encoder_cpr`, a mandatory CLI argument).
Must run **before** `commutation learn` below -- that routine's own
alignment math already assumes `counts_per_elec_rev` is correct.

A Z-index-based fallback for when CPR isn't known ahead of time (use the Z
index to discover a full mechanical revolution's worth of counts directly,
instead of requiring it as an argument) was designed but deliberately not
implemented -- needs a working Z index and the motor to actually complete
full mechanical revolutions during the test, both real requirements worth
keeping separate from the simpler known-CPR method rather than building
both at once.

## One-time alignment commissioning (`commutation learn`)

Electrical angle is `pole_pairs x mechanical_angle`, offset by wherever the
motor's electrical zero sits relative to the encoder's zero -- not known a
priori. A `polarity learn`-shaped routine (`main.c`'s `AppCommutationLearn`):
drives a small constant open-loop two-phase torque commutated directly off
the live (not yet offset-corrected) Hall sector, watches each sector
transition against the encoder's own continuous angle
(`Commutation.c`'s `CommutationLearnAccumulate()`/`CommutationLearnEvaluate()`,
a circular running mean that unwraps each new sample near the running
estimate so it never has to special-case the 359/0 boundary), and persists
the result (`commutation.offset_deg`). Requires `output.topology=
two_phase_analog` already set; rejected otherwise -- same as
`detect_poles`. Not bench-verified -- same hardware dependency as
everything else here, plus its own open question: the fixed
`COMMUTATION_LEARN_DURATION_TICKS`/`COMMUTATION_LEARN_MIN_TRANSITIONS`
guesses are untuned against any real motor's actual speed under this test
torque.

## Polarity faults can't be detected here either

Commutation angle comes from Hall+encoder, a *position* reference, not an
independent check that the motor is actually producing torque in the
commanded direction. If the alignment offset is wrong (bad commissioning
run, miswired Halls, swapped motor leads), this firmware has no direct way
to notice -- the commutation math runs exactly as commanded, just against
the wrong angle. Unlike a scalar single-command drive where a
reversed-direction failure usually just "runs backward," a wrong-phase
two-phase command can mean driving current with little or no useful
torque output at all -- heating, not visible motion -- see
`docs/primer/why-faults-matter.md` for why that's the worse failure to
miss, not just an equivalent one relocated. `Fault.c`'s
`FAULT_SUSTAINED_CURRENT` (sustained high commanded torque as a proxy,
applies to any topology, not just this one) is the only indirect
mitigation that exists -- run `commutation learn`/`detect_poles` and
`polarity learn` (with encoder feedback) before ever trusting a bring-up
that lacks a direct check.

## Not reintroduced on this branch: a feedback-source rejection rule

Upstream history (the project this firmware grew out of) had a real tach
feedback path and a hard rule rejecting `output.topology=two_phase_analog`
+ `feedback.source=tach` together, since two-phase commutation needs a
live encoder for its angle math and tach feedback's whole point was "no
encoder." This repo has no `feedback.source` axis at all (`main` always
runs on the real encoder) -- so that specific conflict doesn't exist here.
If a future feedback-source branch (e.g. `tach-emulation` or
`tamagawa-feedback`) ever lands on top of this one, revisit whether an
equivalent rejection rule is needed then, rather than guessing at it now.

# AGENTS.md

Guidance for AI agents (and humans) working in this repo. This is a
fresh document for this project, not inherited from anywhere else —
nothing here assumes a parent monorepo or unrelated toolchain
conventions.

## What this is

Closed-loop analog torque-command servo controller firmware
(STM32L152/PlatformIO). The project's one real supported board is the
NUCLEO-L152RE, built and tested on `main`. Hardware paths that haven't
had real bench time (another MCU family, a second dedicated telemetry
UART, two-phase/Hall commutation output, Tamagawa absolute-encoder
feedback), plus one bench-validated feature that needs a design-review
pass before it can land (emulated-tach feedback), live as draft PR
branches instead of behind a build flag in `main` — see *Branch
convention* below.

## Repo layout

- `firmware/` — the PlatformIO project.
  - Build: `pio run -d firmware`
  - Native unit tests (host-side, HAL-independent logic only):
    `pio test -d firmware -e native`
- `docs/` — Sphinx + MyST source (see `docs/conf.py`), built for
  ReadTheDocs; also directly readable as plain Markdown without
  building anything. `make -C docs html` builds locally. **Not tracked
  in git yet** (see `.gitignore`) — written, but held back from this
  first published version rather than shipped half-reviewed. Remove the
  `docs/` line from `.gitignore` and `git add docs/` when it's ready.
- `tools/` — host-side Python: `monitor_frame.py` (the binary telemetry
  frame decoder -- `FrameSync`/`decode_frame`/`crc8`; stdlib-only on
  purpose, so anything that just wants to decode frames doesn't have to
  pull in `monitor_viewer.py`'s pygame dependency), `monitor_viewer.py`
  (live telemetry viewer, built on `monitor_frame.py`), `psweep_tune.py`/
  `confirm_point.py` (tuning search tools, on top of `scorecard_core.py`/
  `scorecard_session.py`'s shared recorder/session infra, which also
  needs `scorecard_control.py`), `capture_moves.py`/`plot_telemetry.py`
  (a small repeatable move sequence + a styled position/velocity/torque
  chart of it -- also built on `monitor_frame.py`), `bench_report.py`
  (turns a recorder directory into a `report.md` + plot — run this
  instead of hand-writing bench-results write-ups), `serial_console.py`
  (generic CLI), `sigrok/export_run.py` + `sigrok/capture_usb_concurrency.py`
  + `sigrok-decoder-monitor/` (binary telemetry capture/decode/export).

## Branch convention

Unverified/no-bench-hours hardware paths currently live as draft PRs off
`main`:

- `monitor-usart` — a second, dedicated UART for binary telemetry,
  independent of composite USB.
- `l4` — STM32L431 support (base: `monitor-usart`, since L431 has no USB
  at all).
- `two-phase-output` — Hall-commutated two-phase analog output.
- `f103` — STM32F103 ("Blue Pill") support, PWM+RC output stage (no
  on-chip DAC on that part).
- `tamagawa-feedback` — Tamagawa absolute-encoder feedback (bring-up
  stage, not a port of working code — the protocol decode itself is
  still unbuilt).
- `tach-emulation` — decimated/emulated-tach velocity feedback (bench-
  validated already, unlike the others above — it's a draft because
  merging it means revisiting `main`'s "feedback is always the encoder"
  framing repo-wide, not because of missing hardware or bench time).

`main` only ever ships what's actually been run on real hardware. A
branch merges into `main` only after it clears the same bar — bench
time, not just a clean build — that everything already in `main` has
cleared.

## ConfigStore / flash convention

`CONFIG_FLASH_VERSION` (`firmware/src/ConfigStore.c`) starts at `1`.
Bump it on any field-layout change. **No migration**: an old-version
record is simply rejected on load and firmware defaults apply instead —
don't write migration code for this, that's a deliberate simplicity
choice, not an oversight.

## Documentation convention

Mark anything not bench-verified explicitly, in the doc itself (e.g.
"not yet bench-validated," "written, not yet run against real
hardware") — this is the project's established house style, carried
through every doc in `docs/`. Don't narrate deleted or superseded
subsystems in permanent prose; git history carries that now. A doc that
turns out to describe something removed should be edited to drop the
reference, not padded with "this used to exist, here's why it's gone."

## Units

`position.kp`, `velocity.kp`, `velocity.ki` are Q8.8 fixed-point (`256`
== `1.0`). `torque.command`, `torque.min`/`.max` are a *different*,
plain signed integer scale, full scale `±32767` — not Q8.8, not
volts/amps/N·m. Mixing the two up is the easy mistake; see
`docs/architecture.md`'s *Units* section for the full explanation before
touching any gain/clamp code.

## Attribution

No AI/Claude attribution in commit messages or PR descriptions.

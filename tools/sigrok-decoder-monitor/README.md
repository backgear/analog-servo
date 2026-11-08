# analog-servo Monitor protocol sigrok decoder

The protocol itself (frame layout, transports, on/off commands) is
documented in [`../../docs/monitor-protocol.md`](../../docs/monitor-protocol.md) —
this file only covers the decoder.

## The sigrok decoder (`analog_servo_monitor/`)

A libsigrokdecode protocol decoder that stacks on sigrok's built-in `uart`
PD and parses the frame above into annotations in PulseView.

**Status: drafted, not run against a real capture yet.** Two things worth
knowing before you trust it:

- It assumes a specific shape for the `uart` PD's Python output (one
  `('DATA', rxtx, (value, ...))` tuple per byte, RX = index 0) — check that
  against your installed sigrok's `decoders/uart/pd.py` if it doesn't pick
  up any bytes at all; see `pd.py`'s top comment for exactly what to adjust.
- The frame carries a real CRC-8 (same poly/init as `Monitor.c`'s `Crc8()`,
  reimplemented in `pd.py`'s `crc8()`), so the decoder can tell a real frame
  from coincidental bytes that happen to start with `0xAA` with much better
  than 1-in-256 confidence — not the old range-check-only plausibility
  filter. A sliding window still checks every byte position as a candidate
  frame start (not just "the next one after a failed lock"), so a false
  lock on a payload byte that happens to equal `0xAA` self-corrects within
  at most one frame instead of losing alignment for good.

**Install**: copy (or symlink) `analog_servo_monitor/` into your
libsigrokdecode decoders directory (e.g.
`~/.local/share/libsigrokdecode/decoders/` on Linux), then in PulseView
stack it on top of a `uart` decoder pointed at the captured CDC byte stream.

## Native waveform channels

This decoder produces annotations, not native numeric waveform channels.
For mixed analog/digital PulseView options and a verified recorded-data `.sr`
example, see [the integration notes](../../docs/sigrok-integration.md).

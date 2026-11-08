#!/usr/bin/env python3
"""Capture TICK_ACTIVE/SIGNAL_BUSY alongside real composite-USB CLI/telemetry
traffic, for the bench demonstration in docs/examples/usb-concurrency-trace.md.

Wraps sigrok-cli (fx2lafw driver) to record CH0=TICK_ACTIVE_PIN (PC10),
CH1=SIGNAL_BUSY_PIN (PB13) into a timestamped .sr file, while driving a few
CLI round-trips (and optionally toggling `monitor start`/`stop`) against the
board through tools/serial_console.py's socket interface and logging each
request's host-side timestamp to a sidecar JSON file next to the .sr. The
correlation between the two files -- not the .sr capture alone -- is the
actual proof described in that doc.

UNEXERCISED AGAINST REAL HARDWARE: written against sigrok-cli's documented
fx2lafw options and serial_console.py's documented --socket/--send interface,
not run against a real board or a real logic analyzer in this environment.
Treat any error message or flag name here as a starting point to debug
against the real tools, not as bench-confirmed behavior.
"""
import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_SAMPLE_RATE_HZ = 1_000_000  # 1 MHz -- resolves a single 1ms tick period
                                     # into ~1000 samples; see the trace doc's
                                     # "what a good trace looks like" section.
DEFAULT_CHANNELS = "D0=tick_active,D1=signal_busy"


def run_capture(duration_s, sample_rate_hz, out_sr, channels):
    cmd = [
        "sigrok-cli",
        "--driver", "fx2lafw",
        "--channels", channels,
        "--config", f"samplerate={sample_rate_hz}",
        "--time", f"{int(duration_s * 1000)}ms",
        "--output-file", str(out_sr),
        "--output-format", "srzip",
    ]
    print(f"[capture] {' '.join(cmd)}", file=sys.stderr)
    return subprocess.Popen(cmd)


def drive_traffic(socket_path, requests, events_out):
    """Send a few CLI round-trips through an already-running serial_console.py
    owner (see docs/serial-console.md's 'Share the existing owner with
    scripts' section), logging a host-side timestamp around each one."""
    events = []
    for req in requests:
        t0 = datetime.now(timezone.utc).isoformat()
        try:
            reply = subprocess.run(
                ["python3", "tools/serial_console.py", "--socket", str(socket_path),
                 "--send", req],
                capture_output=True, text=True, timeout=5,
            )
            out = reply.stdout.strip()
        except Exception as exc:  # noqa: BLE001 -- bench script, report and continue
            out = f"<error: {exc}>"
        t1 = datetime.now(timezone.utc).isoformat()
        events.append({"request": req, "sent_at": t0, "replied_at": t1, "reply": out})
        print(f"[traffic] {req!r} -> {out!r}", file=sys.stderr)
    events_out.write_text(json.dumps(events, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--duration", type=float, default=5.0,
                   help="capture length in seconds (default: 5.0)")
    p.add_argument("--sample-rate", type=int, default=DEFAULT_SAMPLE_RATE_HZ,
                   help=f"logic-analyzer sample rate in Hz (default: {DEFAULT_SAMPLE_RATE_HZ})")
    p.add_argument("--channels", default=DEFAULT_CHANNELS,
                   help=f"sigrok-cli --channels spec (default: {DEFAULT_CHANNELS!r}) -- "
                        "match your analyzer's actual D0/D1 wiring to PC10/PB13")
    p.add_argument("--out-dir", type=Path, default=Path("captures"),
                   help="directory to write the .sr capture and sidecar events file into")
    p.add_argument("--socket", type=Path, default=Path("/tmp/servo-console.sock"),
                   help="serial_console.py owner socket to drive traffic through -- "
                        "start one first: python3 tools/serial_console.py <port> --socket <this path>")
    p.add_argument("--monitor", action="store_true",
                   help="also toggle `monitor start`/`monitor stop` around the capture, "
                        "so binary telemetry is flowing during it too")
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_sr = args.out_dir / f"usb-concurrency-{stamp}.sr"
    out_events = args.out_dir / f"usb-concurrency-{stamp}-events.json"

    requests = ["get axis.enabled", "get position.raw", "get velocity.raw",
                "get torque.command", "status"]
    if args.monitor:
        requests = ["monitor start"] + requests + ["monitor stop"]

    capture = run_capture(args.duration, args.sample_rate, out_sr, args.channels)
    # Give sigrok-cli a moment to start sampling before traffic begins, so the
    # first request isn't racing capture startup.
    time.sleep(0.5)
    drive_traffic(args.socket, requests, out_events)
    capture.wait(timeout=args.duration + 10)

    print(f"Capture: {out_sr}")
    print(f"Events:  {out_events}")
    print("Cross-check against tools/sigrok-decoder-monitor/ for the binary "
          "telemetry frames interleaved in this same window.")


if __name__ == "__main__":
    main()

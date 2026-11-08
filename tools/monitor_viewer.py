#!/usr/bin/env python3
"""Live viewer for torque_servo's Monitor.c binary telemetry -- no sigrok/
PulseView needed. See ../../docs/monitor-protocol.md for the wire format
this decodes, and README.md (this directory) for usage.
"""
import argparse
import collections
import queue
import sys
import threading
import time

import pygame
import serial

from monitor_frame import FRAME_LEN, SYNC_BYTE, MODE_NAMES, FAULT_REASON_NAMES, FrameSync


def reader_thread(ser, out_queue, stats, stop_event):
    sync = FrameSync()
    last_seq = None
    while not stop_event.is_set():
        try:
            data = ser.read(max(1, ser.in_waiting))
        except serial.SerialException as exc:
            stats["error"] = str(exc)
            stop_event.set()
            return
        if not data:
            continue
        stats["bytes"] += len(data)
        for frame in sync.feed(data):
            if last_seq is not None and frame["seq"] != last_seq + 1:
                stats["seq_gaps"] += 1
            last_seq = frame["seq"]
            out_queue.put(frame)


def open_cli_and_toggle_monitor(port, baud, start):
    with serial.Serial(port, baud, timeout=1) as cli:
        cli.write(b"monitor start\r\n" if start else b"monitor stop\r\n")
        cli.flush()
        time.sleep(0.1)


COLOR_BG = (18, 18, 22)
COLOR_FG = (220, 220, 220)
COLOR_OK = (90, 200, 120)
COLOR_WARN = (230, 190, 80)
COLOR_BAD = (220, 80, 80)
COLOR_DIM = (120, 120, 130)


def field_color(key, value):
    if key == "fault_latched":
        return COLOR_BAD if value else COLOR_OK
    if key in ("synthetic", "cycle_start"):
        return COLOR_WARN if value else COLOR_DIM
    if key in ("enabled", "hw_enable"):
        return COLOR_OK if value else COLOR_DIM
    if key == "idle_reduced":
        return COLOR_WARN if value else COLOR_DIM
    return COLOR_FG


DISPLAY_LEN = 200    # samples actually drawn per chart (unchanged width from before) --
                      # DEFAULT only, overridable via --display-len; see run()'s override
                      # below. Must comfortably exceed whatever half_period_ticks a
                      # `signal square` test under observation is using, or the chart
                      # window is physically too short to ever show a full cycle's
                      # reversal -- it'll just look like one-way motion no matter how
                      # cleanly the real (measured) position oscillates underneath.
HISTORY_LEN = 1000   # backing buffer kept around so a trigger point can be found
                      # within it, well before it's actually displayed -- DEFAULT only,
                      # see run()'s override below. Must stay well above DISPLAY_LEN
                      # (find_rising_trigger needs real history beyond the display
                      # window to find a trigger point in) -- run() enforces this.


def find_sync_trigger(cycle_start_list, display_len):
    """Firmware-sourced trigger: find the MOST RECENT tick where Monitor.c's
    status bit7 (cycle_start, see docs/monitor-protocol.md) was set, that
    still leaves a full `display_len`-sample window of newer data after it.
    Same windowing rule as find_rising_trigger() below, but with no level to
    guess -- a `signal square` test marks its own cycle boundary, so this is
    preferred over find_rising_trigger() whenever the stream actually has a
    cycle_start pulse in the searchable range (covers `signal square
    position` too, where torqueCommand is the cascade's output and doesn't
    reliably cross any fixed level at the real cycle boundary at all).
    Returns None if there isn't enough history yet or no pulse is found in
    range (e.g. not currently running a `signal square` test) -- caller
    should fall back to find_rising_trigger(), then a plain scroll."""
    n = len(cycle_start_list)
    if n < display_len:
        return None
    latest_possible = n - display_len
    for i in range(latest_possible, -1, -1):
        if cycle_start_list[i]:
            return i
    return None


def find_rising_trigger(history_list, display_len, level=None):
    """Oscilloscope-style trigger: find the MOST RECENT rising-edge crossing
    of `level` that still leaves a full `display_len`-sample window of newer
    data after it. Re-run every redraw, this picks the same phase point each
    time for a repeating signal (e.g. `signal square torque`), so the
    displayed window holds still instead of a plain scroll where a periodic
    waveform drifts past at an arbitrary, ever-changing phase. `level`
    defaults to the midpoint of the search window's own min/max -- self
    calibrating for a bipolar square wave with no manual threshold needed.
    Returns None (caller should fall back to a plain trailing-window scroll)
    if there isn't enough history yet or no crossing is found at all (e.g. a
    steady DC signal with nothing to trigger on)."""
    n = len(history_list)
    if n < display_len + 2:
        return None
    if level is None:
        lo, hi = min(history_list), max(history_list)
        if lo == hi:
            return None   # flat signal -- no edge to trigger on
        level = (lo + hi) / 2.0
    latest_possible = n - display_len
    for i in range(latest_possible, 1, -1):
        if history_list[i - 1] < level <= history_list[i]:
            return i
    return None


def run(args):
    global DISPLAY_LEN, HISTORY_LEN
    DISPLAY_LEN = args.display_len
    HISTORY_LEN = max(1000, DISPLAY_LEN * 5)   # same "well beyond the display window"
                                                 # margin as the original 1000/200 ratio

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    out_queue = queue.Queue()
    stats = {"bytes": 0, "seq_gaps": 0, "error": None}
    stop_event = threading.Event()
    thread = threading.Thread(target=reader_thread, args=(ser, out_queue, stats, stop_event), daemon=True)

    if args.cli_port:
        open_cli_and_toggle_monitor(args.cli_port, args.cli_baud, start=True)

    thread.start()

    pygame.init()
    pygame.display.set_caption("torque_servo Monitor")
    screen = pygame.display.set_mode((800, 480), pygame.RESIZABLE)
    font = pygame.font.SysFont("monospace", 18)
    big_font = pygame.font.SysFont("monospace", 22, bold=True)
    clock = pygame.time.Clock()

    position_hist = collections.deque(maxlen=HISTORY_LEN)
    velocity_hist = collections.deque(maxlen=HISTORY_LEN)
    torque_hist = collections.deque(maxlen=HISTORY_LEN)
    cmd_position_hist = collections.deque(maxlen=HISTORY_LEN)
    cmd_velocity_hist = collections.deque(maxlen=HISTORY_LEN)
    following_error_hist = collections.deque(maxlen=HISTORY_LEN)   # cmd_position - position,
        # computed here rather than added to the wire frame -- both inputs are already in
        # it (see docs/monitor-protocol.md), and this is the same quantity Control.c's own
        # positionError already is internally, just reconstructed client-side.
    cycle_start_hist = collections.deque(maxlen=HISTORY_LEN)
    # Digital status bits, kept as their own time-aligned histories (same
    # maxlen/append cadence as the analog ones above) so the "tiny digital
    # row" strip below the charts can show each bit's history across
    # exactly the same window -- triggered or trailing -- as the analog
    # traces, not just its instantaneous current value (already shown in
    # the left-hand rows list).
    DIGITAL_BIT_KEYS = ("fault_latched", "synthetic", "idle_reduced", "enabled", "hw_enable", "cycle_start")
    DIGITAL_BIT_LABELS = {"fault_latched": "fault", "synthetic": "synthetic", "idle_reduced": "idle_lo",
                           "enabled": "enabled", "hw_enable": "hw_en", "cycle_start": "sync"}
    digital_hist = {k: collections.deque(maxlen=HISTORY_LEN) for k in DIGITAL_BIT_KEYS}
    trigger_enabled = True   # 't' toggles -- see find_sync_trigger()/find_rising_trigger()

    # Trigger holdoff, same idea as a real scope: a single missed edge (one
    # noisy tick, a brief gap between repeats) shouldn't instantly dump you
    # back to a plain scroll -- hold the last successfully-triggered window
    # on screen and keep retrying before giving up. Overridable via
    # --trigger-hold: the default needs to be long enough to survive the
    # gap between events, not just a single dropped sample -- e.g. autotune
    # refine's triggerable moments (its own direction reversals, not a
    # steady repeating waveform) are seconds apart, not milliseconds.
    TRIGGER_HOLD_SECONDS = args.trigger_hold
    last_trig_windows = None   # (p_window, v_window, t_window, cp_window, cv_window, fe_window, digital_windows) or None
    last_trigger_time = 0.0

    latest = None
    frame_count = 0
    fps_window_start = time.time()
    fps = 0.0

    # Heartbeat dot instead of a raw seq number -- seq only ever mattered as
    # "is this still moving," and a blinking dot shows that at a glance
    # without a number changing 1000x/sec being distracting. Blink rate is
    # capped (HEARTBEAT_MIN_PERIOD) independent of the real frame rate, and
    # the dot freezes off if frames stop arriving entirely.
    HEARTBEAT_MIN_PERIOD = 0.25
    heartbeat_on = False
    last_heartbeat_toggle = 0.0
    last_frame_time = 0.0

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_t:
                    trigger_enabled = not trigger_enabled
                    last_trig_windows = None   # don't hold a stale freeze across a toggle
                    last_trigger_time = 0.0
                # No VIDEORESIZE handling needed -- as of pygame 2.0, the
                # display Surface auto-resizes in place when the window is
                # dragged (screen.get_size() below already reflects it).
                # Calling pygame.display.set_mode() again here -- the old
                # pre-2.0 idiom -- forces SDL to tear down and recreate the
                # actual OS window instead of just resizing it, which is
                # exactly the "disappears for a second, comes back the same
                # size" flicker that calling it here would cause.

            drained = 0
            while True:
                try:
                    latest = out_queue.get_nowait()
                except queue.Empty:
                    break
                position_hist.append(latest["position"])
                velocity_hist.append(latest["velocity"])
                torque_hist.append(latest["torque"])
                cmd_position_hist.append(latest["cmd_position"])
                cmd_velocity_hist.append(latest["cmd_velocity"])
                following_error_hist.append(latest["cmd_position"] - latest["position"])
                cycle_start_hist.append(latest["cycle_start"])
                for k in DIGITAL_BIT_KEYS:
                    digital_hist[k].append(latest[k])
                drained += 1
            frame_count += drained

            now = time.time()
            if drained > 0:
                last_frame_time = now
            if now - last_frame_time > 1.0:
                heartbeat_on = False   # nothing arriving -- stop blinking, read as "dead"
            elif now - last_heartbeat_toggle >= HEARTBEAT_MIN_PERIOD and drained > 0:
                heartbeat_on = not heartbeat_on
                last_heartbeat_toggle = now

            if now - fps_window_start >= 1.0:
                fps = frame_count / (now - fps_window_start)
                frame_count = 0
                fps_window_start = now

            win_w, win_h = screen.get_size()
            screen.fill(COLOR_BG)

            if stats["error"]:
                msg = font.render("Serial error: %s" % stats["error"], True, COLOR_BAD)
                screen.blit(msg, (16, 16))
            elif latest is None:
                msg = font.render("Waiting for frames on %s ..." % args.port, True, COLOR_DIM)
                screen.blit(msg, (16, 16))
            else:
                y = 16
                header = "%.0f frames/s  seq_gaps=%u" % (fps, stats["seq_gaps"])
                screen.blit(big_font.render(header, True, COLOR_FG), (16, y))
                heartbeat_color = COLOR_OK if heartbeat_on else COLOR_DIM
                pygame.draw.circle(screen, heartbeat_color, (win_w - 24, y + 11), 8)
                y += 36

                rows = [
                    ("mode", latest["mode"]),
                    ("position", latest["position"]),
                    ("velocity", latest["velocity"]),
                    ("torque", latest["torque"]),
                    ("enabled", latest["enabled"]),
                    ("hw_enable", latest["hw_enable"]),
                    ("idle_reduced", latest["idle_reduced"]),
                    ("synthetic", latest["synthetic"]),
                    ("cycle_start", latest["cycle_start"]),
                    ("fault_latched", latest["fault_latched"]),
                    ("fault_reason", latest["fault_reason"]),
                ]
                for key, value in rows:
                    color = field_color(key, value)
                    line = "%-16s %s" % (key, value)
                    screen.blit(font.render(line, True, color), (16, y))
                    y += 22

                # PulseView-style: every channel is its own ribbon, full
                # width, stacked top to bottom in ONE column so they all
                # share the exact same x-axis (time) -- no separate
                # cmd/measured columns with their own independent pixel
                # mapping, which made it easy to misjudge whether two
                # events on different charts actually lined up in time.
                # Order (top to bottom) mirrors a scope/logic-analyzer
                # convention: commanded (desired) signals first, then
                # measured (encoder), then digital status bits at the
                # bottom -- see monitor-protocol.md for why commanded vs.
                # measured are different fields, not duplicates.
                chart_area_x0 = 320
                footer_y = win_h - 26
                chart_area_h = max(120, footer_y - 16 - 16)
                chart_x0 = chart_area_x0
                chart_w = max(120, win_w - chart_area_x0 - 16)
                # analog_ribbon_h/digital_h are computed further down, AFTER
                # carving the digital-status band off chart_area_h -- see
                # that comment for why the order matters.

                # Oscilloscope-style trigger: hold the displayed window at a
                # consistent phase of a repeating signal instead of a plain
                # scroll, where the same waveform drifts past at whatever
                # phase it happens to be at each redraw. Prefers Monitor.c's
                # firmware-sourced cycle_start bit (set by `signal square`,
                # see docs/monitor-protocol.md) over guessing a level
                # crossing on torque -- the bit is exact and amplitude-
                # independent, and (unlike torque) also works for `signal
                # square position`, where torque is the cascade's output,
                # not the raw square wave. Falls back to the old
                # level-crossing trigger (useful if you're watching a plain
                # `signal square torque` capture from before this bit
                # existed, or anything else with no cycle_start pulses in
                # range), then to a plain trailing window (original
                # behavior) if 't' has turned triggering off entirely, or
                # neither has anything to trigger on yet. All five charts
                # share the same trig_idx/window so they stay time-aligned
                # with each other, not just internally stable.
                position_list = list(position_hist)
                velocity_list = list(velocity_hist)
                torque_list = list(torque_hist)
                cmd_position_list = list(cmd_position_hist)
                cmd_velocity_list = list(cmd_velocity_hist)
                following_error_list = list(following_error_hist)
                cycle_start_list = list(cycle_start_hist)
                digital_lists = {k: list(digital_hist[k]) for k in DIGITAL_BIT_KEYS}
                trig_idx = None
                trig_source = None
                if trigger_enabled:
                    trig_idx = find_sync_trigger(cycle_start_list, DISPLAY_LEN)
                    trig_source = "sync"
                    if trig_idx is None:
                        trig_idx = find_rising_trigger(torque_list, DISPLAY_LEN)
                        trig_source = "torque"
                if trig_idx is not None:
                    p_window = position_list[trig_idx:trig_idx + DISPLAY_LEN]
                    v_window = velocity_list[trig_idx:trig_idx + DISPLAY_LEN]
                    t_window = torque_list[trig_idx:trig_idx + DISPLAY_LEN]
                    cp_window = cmd_position_list[trig_idx:trig_idx + DISPLAY_LEN]
                    cv_window = cmd_velocity_list[trig_idx:trig_idx + DISPLAY_LEN]
                    fe_window = following_error_list[trig_idx:trig_idx + DISPLAY_LEN]
                    digital_windows = {k: digital_lists[k][trig_idx:trig_idx + DISPLAY_LEN] for k in DIGITAL_BIT_KEYS}
                    last_trig_windows = (p_window, v_window, t_window, cp_window, cv_window, fe_window, digital_windows)
                    last_trigger_time = now
                    trig_label = "trig: %s" % trig_source
                elif (trigger_enabled and last_trig_windows is not None and
                      (now - last_trigger_time) < TRIGGER_HOLD_SECONDS):
                    # Missed this redraw's edge (one noisy sample, a brief
                    # gap between repeats of a periodic signal) -- hold the
                    # last good triggered window frozen rather than
                    # instantly dropping to a plain scroll; still retries
                    # every frame above, so a real re-trigger wins the
                    # instant one shows up.
                    p_window, v_window, t_window, cp_window, cv_window, fe_window, digital_windows = last_trig_windows
                    trig_label = "trig: holding"
                else:
                    p_window = position_list[-DISPLAY_LEN:]
                    v_window = velocity_list[-DISPLAY_LEN:]
                    t_window = torque_list[-DISPLAY_LEN:]
                    cp_window = cmd_position_list[-DISPLAY_LEN:]
                    cv_window = cmd_velocity_list[-DISPLAY_LEN:]
                    fe_window = following_error_list[-DISPLAY_LEN:]
                    digital_windows = {k: digital_lists[k][-DISPLAY_LEN:] for k in DIGITAL_BIT_KEYS}
                    trig_label = "trig: off" if not trigger_enabled else "trig: searching"

                # Carve a thin band for the digital status rows off the
                # bottom of the chart area -- see draw_digital_rows().
                # Comes off chart_area_h itself (not a separate reserved
                # region computed above) so the analog ribbons above
                # automatically shrink to make room as the window is
                # resized, same as every other chart dimension here.
                digital_h = max(60, min(90, chart_area_h // 4))
                chart_area_h -= digital_h + 8
                analog_ribbon_h = chart_area_h // 6

                analog_ribbons = [
                    (cp_window, "cmd position", False),
                    (cv_window, "cmd velocity", False),
                    (p_window, "position", False),
                    (v_window, "velocity", False),
                    (fe_window, "following error", True),   # cmd_position - position --
                        # zero-centered like torque, same reasoning: its sign is the point.
                    (t_window, "torque (%s)" % trig_label, True),
                ]
                for i, (window, label, zero_centered) in enumerate(analog_ribbons):
                    draw_strip_chart(screen, window, (chart_x0, 16 + i * analog_ribbon_h),
                                      (chart_w, analog_ribbon_h - 8), label, zero_centered=zero_centered)

                digital_y = 16 + chart_area_h + 8
                draw_digital_rows(screen,
                                   [(k, DIGITAL_BIT_LABELS[k], digital_windows[k]) for k in DIGITAL_BIT_KEYS],
                                   (chart_x0, digital_y), (chart_w, digital_h))

            footer = font.render("q/esc to quit, t to toggle trigger" + (" -- sending 'monitor stop' on exit" if args.cli_port else ""), True, COLOR_DIM)
            screen.blit(footer, (16, win_h - 26))

            pygame.display.flip()
            clock.tick(60)
    finally:
        stop_event.set()
        thread.join(timeout=1)
        ser.close()
        if args.cli_port:
            open_cli_and_toggle_monitor(args.cli_port, args.cli_baud, start=False)
        pygame.quit()


def draw_strip_chart(screen, history, top_left, size, label, zero_centered=False):
    """zero_centered=True draws DC-coupled, scope style: 0 is pinned to the
    chart's vertical center and the scale is symmetric (+-max(|value|)),
    instead of autoscaling to the window's own min/max (which floats
    wherever 0 happens to land, and for a signal whose whole point is its
    sign -- torque -- makes it hard to read at a glance which side of zero
    you're on). A thin reference line marks the zero center so it's not
    just implied."""
    x0, y0 = top_left
    w, h = size
    pygame.draw.rect(screen, (30, 30, 36), (x0, y0, w, h))
    pygame.draw.rect(screen, COLOR_DIM, (x0, y0, w, h), width=1)
    font = pygame.font.SysFont("monospace", 14)
    screen.blit(font.render(label, True, COLOR_DIM), (x0 + 4, y0 + 2))

    if len(history) < 2:
        return
    if zero_centered:
        amp = max(max(abs(v) for v in history), 1)
        lo, hi = -amp, amp
        zero_py = y0 + h // 2
        pygame.draw.line(screen, COLOR_DIM, (x0, zero_py), (x0 + w, zero_py), width=1)
    else:
        lo, hi = min(history), max(history)
        if lo == hi:
            lo -= 1
            hi += 1
    span = hi - lo
    points = []
    n = len(history)
    for i, value in enumerate(history):
        px = x0 + int(i * w / (n - 1))
        py = y0 + h - int((value - lo) / span * (h - 4)) - 2
        points.append((px, py))
    pygame.draw.lines(screen, COLOR_OK, False, points, width=2)


def draw_digital_rows(screen, rows, top_left, size):
    """Tiny logic-analyzer-style strip for the frame's boolean status bits
    (fault/autolearn/etc, see docs/monitor-protocol.md's status byte) --
    time-aligned with whatever window (triggered or trailing) the analog
    charts above are showing, via the same history slices the caller
    already sliced for them. `rows` is a list of (field_key, label,
    history) triples -- field_key picks the "lit" color via field_color()'s
    own convention (so e.g. the fault row reads red, same as the text row
    above it), label is what's drawn in the corner. Each lit run is one
    filled rect (not one draw call per pixel) -- cheap regardless of how
    many raw samples are packed into the row's width, and reads as clean
    digital edges rather than a blur."""
    x0, y0 = top_left
    w, h = size
    n_rows = len(rows)
    if n_rows == 0 or w <= 0 or h <= 0:
        return
    row_h = max(1, h // n_rows)
    font = pygame.font.SysFont("monospace", 12)
    for r, (field_key, label, history) in enumerate(rows):
        ry0 = y0 + r * row_h
        pygame.draw.rect(screen, (30, 30, 36), (x0, ry0, w, row_h))
        n = len(history)
        if n >= 1:
            on_color = field_color(field_key, True)
            run_start = None
            for px in range(w + 1):
                idx = int(px * (n - 1) / (w - 1)) if (px < w and w > 1) else min(px, n - 1)
                lit = px < w and bool(history[idx])
                if lit and run_start is None:
                    run_start = px
                elif not lit and run_start is not None:
                    pygame.draw.rect(screen, on_color, (x0 + run_start, ry0 + 1, px - run_start, row_h - 2))
                    run_start = None
        pygame.draw.rect(screen, COLOR_DIM, (x0, ry0, w, row_h), width=1)
        screen.blit(font.render(label, True, COLOR_FG), (x0 + 4, ry0 + 1))


def main():
    if "--scorecard" in sys.argv:
        from tuning_scorecard import main as scorecard_main
        return scorecard_main([arg for arg in sys.argv[1:] if arg != "--scorecard"])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial device for the Monitor binary telemetry port "
                                      "(e.g. /dev/ttyACM1, or the L431's Monitor UART adapter)")
    parser.add_argument("--baud", type=int, default=2500000,
                         help="baud rate (ignored by USB CDC-ACM in practice, but required by "
                              "pyserial; must match MONITOR_UART_BAUD_RATE for the dedicated "
                              "monitor-UART transport -- default 2500000 is the L431's rate; "
                              "pass 2000000 for the L152, which runs USART1 at a slower clock "
                              "and can't actually hit 2.5M, see docs/monitor-protocol.md)")
    parser.add_argument("--cli-port", default=None,
                         help="optional: CLI port to send 'monitor start'/'monitor stop' to "
                              "automatically on launch/exit, instead of running those commands "
                              "yourself in a separate terminal")
    parser.add_argument("--cli-baud", type=int, default=115200,
                         help="baud rate for --cli-port (default 115200, the CLI's normal rate)")
    parser.add_argument("--display-len", type=int, default=DISPLAY_LEN,
                         help="samples shown per strip chart (default %(default)s, i.e. "
                              "%(default)sms @ 1kHz). Must exceed the half_period_ticks of "
                              "whatever `signal square` test you're watching, or the chart "
                              "window is too short to ever show a full cycle -- e.g. pass "
                              "2200 for `signal square torque <amp> 2000`")
    parser.add_argument("--trigger-hold", type=float, default=10.0,
                         help="seconds to keep the last triggered window frozen on screen "
                              "after the most recent trigger, before falling back to a plain "
                              "scroll (default %(default)s). Raise this for a source whose "
                              "triggerable moments are seconds apart rather than a steady "
                              "repeating waveform -- e.g. autotune refine's direction "
                              "reversals -- so the display doesn't drop to scrolling between them")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()

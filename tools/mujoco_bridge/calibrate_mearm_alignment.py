"""Interactive pose-anchored alignment between the REAL sensors and the
MeArm MuJoCo model (mearm_scene.xml).

Why this exists (2026-09-25): run_demo_live.py's existing calibration
(BASELINE/FORWARD/LEFT_TWIST -> oblique basis) only answers "what human
angle is the sensor reading?". (Update 2026-09-25: this tool no longer uses
that oblique decode -- it leaked twist into pitch -- but mearm_pathb's
spherical tilt/azimuth one, the same as the offline default.) The other half -- "which MeArm pose should
that angle produce?" -- was a full-anatomical-ROM rescale() that compressed
a real ~90deg arm raise into ~14deg of model motion, left the base off
center, and left elbow/claw polarity as a guess. Here the MODEL shows a
target pose, the user copies it with their real arm, and the decoded sensor
values at that moment become the measured calibration: the raw HANG/FORWARD/
LEFT/RIGHT vectors (tilt and azimuth anchors are derived from them by
mearm_pathb.Calibration) and the elbow's real (bend -> model ctrl) anchors.
Polarity, range and center all come from the captured data; the two conventions only eyes can judge (which elbow
direction looks like a bend, which claw end is "closed") are asked as y/n
while looking at the model.

Sequence: HANG -> FORWARD -> LEFT_TWIST -> RIGHT_TWIST (these 4 build the
spherical calibration; a bad one is retried, not saved) -> ELBOW_FLEX -> claw
y/n -> MID_RAISE (a held-out check, NOT used in the fit) -> save -> the
model follows the live arm so it can be checked by eye too.

Only the new "mearm_alignment" key is written (the prior file is backed up
first anyway). Every key the humanoid path uses -- baseline_raw, forward_raw,
left_twist_raw, right_twist_raw, zero_elbow, emg_threshold -- is left exactly
as it was; this run's own raw captures are stored inside "mearm_alignment".
run_demo_live.py --mearm uses it automatically when present.

Must run as `mjpython calibrate_mearm_alignment.py` (needs the viewer; same
main-thread requirement as run_demo_live.py). The board must be running
phase3_control_loop (not a servo test firmware) and power-cycled after
flashing.
"""

import argparse
import math
import shutil
import sys
import threading
import time
import traceback
from pathlib import Path

import mujoco
import mujoco.viewer
import serial

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO_ROOT / "tools"))
import mearm_pathb as pb  # noqa: E402
import sensor_health  # noqa: E402
import run_demo_live as rdl  # noqa: E402
from usb_serial_port import CP2102_PORT, autodetect_port  # noqa: E402

RECORD_SECONDS = 6.0        # same as run_demo_live.py's calibration capture
SETTLE_TAIL_SECONDS = 2.5   # only the last part is averaged (arm needs time to settle)
MIN_TILT_DEG = 20.0         # same guard as run_demo_live.py's MIN_CALIBRATION_TILT_DEG
MIN_ELBOW_DELTA_RAD = 0.6   # ~35deg: flex must differ from straight by at least this
MAX_RETRIES = 3
WAIT_HINT_SECONDS = 5.0     # first 'still waiting' hint; repeated every 2x this
# Refuse to start (and to record) on faulty sensor data -- tools/sensor_health.py, SESSION_LOG 2026-09-28: the 9/27
# calibration was recorded while a sensor was failing and silently saved garbage. None = wait as long as it takes.
HEALTH_PREFLIGHT_MAX_S = None
HEALTH_REPEAT_WARNING_S = 3.0

# Model target poses shown to the user (MeArm ctrl values). Shoulder ctrl's
# high end is the LOWEST arm elevation and its low end the HIGHEST (measured
# 2026-09-24: elevation 43deg at +0.898, 87deg at -0.141); elbow ctrl's low
# end is EXTENDED and high end FOLDED (interior angle ~110deg vs ~40deg).
SH_LOW_ELEV = rdl.MEARM_SHOULDER_CTRL_RANGE[1]
SH_HIGH_ELEV = rdl.MEARM_SHOULDER_CTRL_RANGE[0]
SH_MID = 0.5 * (SH_LOW_ELEV + SH_HIGH_ELEV)
EL_EXTENDED = rdl.MEARM_ELBOW_CTRL_RANGE[0]
EL_FOLDED = rdl.MEARM_ELBOW_CTRL_RANGE[1]
BASE_SWING = 0.9            # rad of model base rotation shown for LEFT/RIGHT (limit is 1.08)
CLAW_LOW, CLAW_HIGH = rdl.MEARM_CLAW_CTRL_RANGE


class PoseState:
    """Shared between the interactive thread (sets what the model should
    show / switches to live-follow) and the main thread (steps the sim and
    renders)."""

    def __init__(self):
        self._lock = threading.Lock()
        self.mode = "target"                       # "target" | "live"
        self.target = (0.0, SH_LOW_ELEV, EL_EXTENDED, CLAW_LOW)   # base, shoulder, elbow, claw
        self.live = None                           # callable -> (base, shoulder, elbow, claw)
        self.abort = False

    def show(self, base, shoulder, elbow, claw):
        with self._lock:
            self.mode = "target"
            self.target = (base, shoulder, elbow, claw)

    def go_live(self, fn):
        with self._lock:
            self.live = fn
            self.mode = "live"

    def snapshot(self):
        with self._lock:
            return self.mode, self.target, self.live, self.abort

    def request_abort(self):
        with self._lock:
            self.abort = True


def wait_for_valid_data(latest):
    started = time.monotonic()
    next_hint = WAIT_HINT_SECONDS
    while not latest.is_ready():
        time.sleep(0.05)
        if time.monotonic() - started >= next_hint:
            print(f"still no live data after {next_hint:.0f}s -- is phase3_control_loop the "
                  f"firmware currently flashed, and was the board power-cycled after flashing?")
            next_hint += 2 * WAIT_HINT_SECONDS
    # is_ready() only means the tick line's core fields arrived; shoulder_raw
    # is a separate match and can still be firmware's (0,0,0) default.
    started = time.monotonic()
    next_hint = WAIT_HINT_SECONDS
    while True:
        raw = latest.snapshot_shoulder_raw()
        if raw[0] is not None and math.hypot(*raw) > 1e-6:
            break
        time.sleep(0.05)
        if time.monotonic() - started >= next_hint:
            print(f"tick lines arrive but the upper-arm IMU vector is missing or all-zero after "
                  f"{next_hint:.0f}s -- is the shoulder MPU6050 (0x68) connected and answering?")
            next_hint += 2 * WAIT_HINT_SECONDS
    # Automatic sensor-health check before anything is recorded: wait, saying what is wrong, until both are healthy.
    monitor = sensor_health.HealthMonitor()
    poller = sensor_health.LatestSamplePoller(latest, monitor)
    started = time.monotonic()
    next_warning = started + 0.5
    while True:
        now = time.monotonic()
        poller.poll(now)
        report = monitor.report(now)
        if sensor_health.ready_to_start(report, now - started):
            print(sensor_health.format_warning(report))
            return
        if HEALTH_PREFLIGHT_MAX_S is not None and now - started >= HEALTH_PREFLIGHT_MAX_S:
            raise RuntimeError(sensor_health.format_warning(report))
        if now >= next_warning and sensor_health.should_announce(report, now - started):
            print(sensor_health.format_warning(report) + "\n   (continues by itself once fixed; no restart needed)")
            next_warning = now + HEALTH_REPEAT_WARNING_S
        time.sleep(0.005)


class CaptureInterrupted(Exception):
    """The live data went quiet during a capture window -- retry the pose."""


def capture_window(latest):
    """Mirrors run_demo_live.py main()'s nested _capture_window (that one
    is a closure inside main(), so it can't be imported): record for
    RECORD_SECONDS, average only the settled tail, warn on residual drift.
    Returns (shoulder_raw_avg, elbow_avg)."""
    samples = []
    t0 = time.monotonic()
    last_print = -1.0
    stalled = False
    while time.monotonic() - t0 < RECORD_SECONDS:
        # A dead link keeps returning the LAST sample, which looks perfectly valid:
        # averaging it would silently save a garbage calibration.
        is_stale, port_error = latest.status()
        if port_error is not None:
            raise RuntimeError(f"serial port failed: {port_error} (the USB-serial adapter was "
                               f"likely unplugged -- reconnect and re-run the calibration)")
        if is_stale:
            stalled = True
            time.sleep(0.02)
            continue
        raw = latest.snapshot_shoulder_raw()
        bad = sensor_health.implausible_problems(raw, latest.snapshot_elbow_raw())
        if bad:
            # a failing sensor during a recording: this attempt is void (a frozen value LOOKS valid -- 9/27)
            raise CaptureInterrupted(sensor_health.format_warning(sensor_health.Report(False, bad)))
        if raw[0] is not None and math.hypot(*raw) > 1e-6:
            t = time.monotonic() - t0
            samples.append((t, raw, latest.snapshot()[3]))
            if t - last_print >= 0.5:
                print(f"    t={t:4.1f}s upperarm raw=({raw[0]:+.3f},{raw[1]:+.3f},{raw[2]:+.3f})")
                last_print = t
        time.sleep(0.02)
    if stalled:
        raise CaptureInterrupted("data interrupted during recording (the firmware stopped sending for a moment)")
    if not samples:
        raise RuntimeError("no valid shoulder samples arrived during the capture window")
    t_max = samples[-1][0]
    tail = [s for s in samples if s[0] >= t_max - SETTLE_TAIL_SECONDS] or samples[-5:]
    n = len(tail)
    raw_avg = tuple(sum(s[1][i] for s in tail) / n for i in range(3))
    elbow_avg = sum(s[2] for s in tail) / n
    if n >= 4:
        t_mid = (tail[0][0] + tail[-1][0]) / 2.0
        first = [s[1] for s in tail if s[0] < t_mid]
        second = [s[1] for s in tail if s[0] >= t_mid]
        if first and second:
            fh = [sum(v[i] for v in first) / len(first) for i in range(3)]
            sh = [sum(v[i] for v in second) / len(second) for i in range(3)]
            drift = max(abs(sh[i] - fh[i]) for i in range(3))
            if drift > 0.05:
                print(f"  Warning: upper-arm raw was still drifting during the settled part (halves differ by up to "
                      f"{drift:.3f} g): the pose may not have been held still. Consider re-recording.")
    return raw_avg, elbow_avg


def capture_pose(state, latest, target, instruction, accept=None):
    """Shows `target` on the model, then Enter -> 3/2/1 -> record. `accept`
    (optional) is called with (raw, elbow) and returns None if fine or a
    Chinese reason string to redo the pose (up to MAX_RETRIES times) --
    same idea as run_demo_live.py's too-small-tilt retry guard."""
    for attempt in range(MAX_RETRIES):
        state.show(*target)
        print("\n" + instruction)
        print("(The model now shows the target pose; match it with your arm.) Press Enter when ready.")
        input()
        print(f"Starting in 3 s. Hold the pose until recording ends ({RECORD_SECONDS:.0f} s in total; "
              f"only the last {SETTLE_TAIL_SECONDS:.1f} s is averaged).")
        for n in (3, 2, 1):
            print(f"  {n}...", flush=True)
            time.sleep(1.0)
        print("Recording! Hold the pose.")
        try:
            raw, elbow = capture_window(latest)
        except CaptureInterrupted as exc:
            print(f"  {exc} (attempt {attempt + 1}/{MAX_RETRIES}): this recording is discarded, please try again.")
            continue
        reason = accept(raw, elbow) if accept else None
        if reason is None:
            print("  Recording accepted.")
            return raw, elbow
        print(f"  {reason} (attempt {attempt + 1}/{MAX_RETRIES}): please try again.")
    raise RuntimeError("the same pose failed too many times; calibration aborted. Check that the sensors are not loose "
                       "or crooked.")


def ask_yes_no(question):
    while True:
        ans = input(f"{question} (y/n): ").strip().lower()
        if ans in ("y", "n"):
            return ans == "y"


def run_flow(state, latest, args):
    wait_for_valid_data(latest)
    print("Live data received. Starting the interactive calibration: the model window shows each target pose.")

    # 1. HANG
    hang_raw, hang_elbow = capture_pose(
        state, latest, (0.0, SH_LOW_ELEV, EL_EXTENDED, CLAW_LOW),
        "[1/7] HANG: arm hanging naturally, elbow straight, hand relaxed.")

    def far_from_hang(raw, _e):
        tilt = rdl.calibration_tilt_deg(hang_raw, raw)
        return None if tilt >= MIN_TILT_DEG else f"only {tilt:.1f}° from HANG (needs >= {MIN_TILT_DEG:.0f}°): movement too small"

    # 2. FORWARD
    fwd_raw, _ = capture_pose(
        state, latest, (0.0, SH_HIGH_ELEV, EL_EXTENDED, CLAW_LOW),
        "[2/7] FORWARD: back to hanging first, then, elbow straight, raise the arm forward to horizontal without "
        "turning the wrist.", far_from_hang)

    # 3. LEFT_TWIST
    left_raw, _ = capture_pose(
        state, latest, (BASE_SWING, SH_MID, EL_EXTENDED, CLAW_LOW),
        "[3/7] LEFT_TWIST: back to hanging first, then, elbow straight, swing the arm fully left, turning the thumb up.", far_from_hang)

    # 4. RIGHT_TWIST. The whole decode is Path B's spherical (tilt, azimuth) one
    # (mearm_pathb.Calibration): the earlier oblique basis leaked twist into
    # 'pitch' (SESSION_LOG 2026-09-24). Building the Calibration is also the
    # validity check -- FORWARD far enough from HANG, LEFT/RIGHT on opposite
    # sides of FORWARD -- so a bad capture is retried, not saved.
    def right_ok(raw, e):
        msg = far_from_hang(raw, e)
        if msg:
            return msg
        try:
            pb.Calibration(hang_raw, fwd_raw, left_raw, raw, hang_elbow)
        except ValueError as exc:
            return f"swing right cannot be told apart from swing left, or points the wrong way ({exc})"
        return None

    right_raw, _ = capture_pose(
        state, latest, (-BASE_SWING, SH_MID, EL_EXTENDED, CLAW_LOW),
        "[4/7] RIGHT_TWIST: back to hanging first, then, elbow straight, swing the arm fully right, turning the thumb down.", right_ok)
    cal = pb.Calibration(hang_raw, fwd_raw, left_raw, right_raw, hang_elbow)
    print(f"\nBasis built: FORWARD is {math.degrees(cal.tilt_forward):.0f}° from hanging, "
          f"left swing azimuth {math.degrees(cal.az_left):+.0f}°, right swing azimuth {math.degrees(cal.az_right):+.0f}°")

    # 5. ELBOW_FLEX
    def flex_ok(_raw, elbow):
        d = abs(elbow - hang_elbow)
        return None if d >= MIN_ELBOW_DELTA_RAD else f"the elbow reading moved only {d:.2f} rad from HANG (needs >= {MIN_ELBOW_DELTA_RAD}): not bent enough"

    _flex_raw, flex_elbow = capture_pose(
        state, latest, (0.0, SH_LOW_ELEV, EL_FOLDED, CLAW_LOW),
        "[5/7] ELBOW_FLEX: arm hanging, bend the elbow fully (hand as close to the shoulder as you can), nothing else "
        "moves.", flex_ok)
    fold_looks_like_bend = ask_yes_no(
        "Look at the model: does its folded forearm mean the same thing as the elbow bend you just made?")
    y_straight, y_flex = (EL_EXTENDED, EL_FOLDED) if fold_looks_like_bend else (EL_FOLDED, EL_EXTENDED)

    # 6. claw polarity (visual only)
    state.show(0.0, SH_LOW_ELEV, EL_EXTENDED, CLAW_HIGH)
    print("\n[6/7] Claw: look at the model's claw (now at the HIGH end of its range).")
    time.sleep(1.5)
    high_is_closed = ask_yes_no("Is the claw CLOSED now? (If unsure, rotate the view and zoom in with the mouse.)")
    claw_open, claw_closed = (CLAW_LOW, CLAW_HIGH) if high_is_closed else (CLAW_HIGH, CLAW_LOW)

    # Fit. Shoulder and base need no anchors of their own: the spherical decode
    # already has them (FORWARD's tilt, LEFT/RIGHT's azimuth, all from the
    # captures saved below). What this tool measures beyond the offline default
    # is the elbow's real swing and polarity, and the claw's polarity.
    elbow_anchors = [[hang_elbow, y_straight], [flex_elbow, y_flex]]
    alignment = {
        "decode": "spherical",
        "elbow_anchors": elbow_anchors,
        "claw_open_ctrl": claw_open,
        "claw_closed_ctrl": claw_closed,
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    # 7. MID_RAISE: held out -- not used in the fit above.
    mid_raw, _ = capture_pose(
        state, latest, (0.0, SH_MID, EL_EXTENDED, claw_open),
        "[7/7] MID_RAISE (a check, not used in the fit): elbow straight, raise the arm forward to about HALF height "
        "(about 45°).")
    tilt_mid, _az = cal.decode(mid_raw)
    frac = tilt_mid / cal.tilt_forward
    verdict = "PASS" if 0.2 <= frac <= 0.8 else "WARN"
    print(f"\nCheck: the half-raise pose sits at {frac * 100:.0f}% of the way from hanging to FORWARD "
          f"(expected about 50%, allowed 20-80%) -> {verdict}")
    if verdict == "WARN":
        print("  The half-raise is far off: an anchor pose may be wrong (often FORWARD or HANG was not reached). The "
              "calibration is still saved; check the live model by eye and rerun if it looks wrong.")

    # Save (back up first)
    calib_path = args.calibration_file
    if calib_path.exists():
        backup = calib_path.with_name(calib_path.name + time.strftime(".bak-%Y%m%d-%H%M%S"))
        shutil.copy2(calib_path, backup)
        print(f"Previous calibration file backed up to {backup.name}")
    # Path A (--humanoid) owns baseline_raw/forward_raw/left_twist_raw/
    # right_twist_raw/zero_elbow/captured_at in this file: NOT touched here
    # (explicit instruction 2026-09-25 -- the humanoid path must stay exactly
    # as it was). This run's own raw captures live INSIDE mearm_alignment
    # instead, and run_demo_live.py --mearm builds its calibration from those, so
    # the elbow anchors and the vectors that produced them can never disagree.
    alignment["captures"] = {
        "hang_raw": list(hang_raw),
        "forward_raw": list(fwd_raw),
        "left_twist_raw": list(left_raw),
        "right_twist_raw": list(right_raw),
        "hang_elbow": hang_elbow,
    }
    rdl.save_calibration_fields(calib_path, {"mearm_alignment": alignment})
    print(f"Saved to {calib_path}")

    def live():
        grip, _p, _r, elbow = latest.snapshot()
        base, shoulder, elbow_ctrl = pb.ctrl_from_sensors(
            cal, latest.snapshot_shoulder_raw(), elbow, elbow_anchors=elbow_anchors)
        return (base, shoulder, elbow_ctrl, rdl.rescale(grip, 0.0, 1.0, claw_open, claw_closed))

    state.go_live(live)
    print("\nCalibration done. The model now follows your arm live: check by eye that hanging, raising forward, "
          "swinging left/right and bending the elbow all match. Close the window to finish.")


def flow_thread_main(state, latest, args):
    try:
        run_flow(state, latest, args)
    except Exception:
        traceback.print_exc()
        state.request_abort()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial device path (default: auto-detect FT232RL)")
    parser.add_argument("--cp2102", action="store_true", help=f"use the CP2102 adapter at {CP2102_PORT}")
    parser.add_argument("--baud", type=int, default=rdl.DEFAULT_BAUD)
    parser.add_argument("--calibration-file", type=Path,
                        default=REPO_ROOT / "tools" / "mujoco_bridge" / "shoulder_calibration.json")
    args = parser.parse_args()

    port = args.port if args.port else autodetect_port(prefer_cp2102=args.cp2102)
    ser = serial.Serial(port, args.baud, timeout=1)
    print(f"Listening on {port} @ {args.baud} baud -- Ctrl+C to stop")

    latest = rdl.LatestSample()
    threading.Thread(target=rdl.reader_thread_main, args=(ser, latest), daemon=True).start()

    model = mujoco.MjModel.from_xml_path(str(rdl.MEARM_SCENE_XML))
    data = mujoco.MjData(model)
    ids = [model.actuator(n).id for n in ("base", "shoulder", "elbow", "claw")]

    state = PoseState()
    threading.Thread(target=flow_thread_main, args=(state, latest, args), daemon=True).start()

    step_count = 0
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            if step_count % 20 == 0:
                # the flow thread only notices a dead port while it is capturing; in
                # live-follow mode nothing else would, and the model would just freeze
                _stale, port_error = latest.status()
                if port_error is not None:
                    sys.exit(f"\nserial port failed: {port_error}\n"
                             f"(the USB-serial adapter was likely unplugged -- reconnect and re-run)")
            mode, target, live, abort = state.snapshot()
            if abort:
                print("Calibration aborted.")
                break
            ctrl = live() if (mode == "live" and live) else target
            for actuator_id, value in zip(ids, ctrl):
                data.ctrl[actuator_id] = value
            if mode == "live" and step_count % 1000 == 0:
                print("model ctrl: base={:+.3f} shoulder={:+.3f} elbow={:+.3f} claw={:+.3f}".format(*ctrl))
            step_count += 1
            mujoco.mj_step(model, data)
            viewer.sync()
            wait = model.opt.timestep - (time.time() - step_start)
            if wait > 0:
                time.sleep(wait)


if __name__ == "__main__":
    main()

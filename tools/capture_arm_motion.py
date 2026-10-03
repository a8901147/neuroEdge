#!/usr/bin/env python3
"""Interactive recording of REAL arm motion from both MPU6050s, to choose the 1-euro filter's two parameters
(include/edgeneuro/filters/vec3_one_euro.hpp: min cutoff, beta) from raw data instead of guessing (SESSION_LOG
2026-10-02).

It walks you through short recordings built around the core demo task, one at a time -- each starts only when you press
Enter, and each can be redone:
  * still holds at every task pose (hanging, forward, left open/gripping, lifted, right, place) -- jitter at rest
  * slow fine aiming at the grasp pose                                                        -- small slow motions
  * the whole 7-step task at demo speed, three times                                          -- must not feel delayed
  * a few poses/motions that are NOT in the task ("check_" phases)                            -- kept out of the tuning

Then it saves everything to a NEW file data/arm_motion_<timestamp>.json (never overwrites) and prints, per phase and
sensor, how fast the arm actually moved (the 1-euro filter's own speed measure: |d(raw accel)/dt| in g/s, about rad/s
for an arm at ~1 g) and the resting noise.

Needs the board running phase3_control_loop (nothing is flashed; the servo-off default build is enough). Close other
programs using the serial port first (run_demo_live.py, watch_imu_raw.py).

    python3 tools/capture_arm_motion.py
    python3 tools/capture_arm_motion.py --port /dev/tty.usbserial-XXXXXXXX
    python3 tools/capture_arm_motion.py --set base_raise    # ~1.5 min: raising vs swinging, for the base (2026-10-04)
"""
import argparse
import json
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
SENSORS = {"upper_arm": ("upper arm", "0x68", "shoulder"), "forearm": ("forearm", "0x69", "elbow")}
# Built around the core demo task (hang -> forward -> left -> grip tape -> lift -> right -> place; SESSION_LOG
# 2026-10-02): still holds at each task pose (jitter depends on the pose -- hanging is where the base direction is least
# stable; gripping adds the ~9-10 Hz physiological tremor measured 2026-09-12), slow fine aiming, the whole task at demo
# speed, and finally poses/motions that are NOT in the task, kept apart as a check against tuning only for the task.
PHASES = [
    ("hold_hang", "Still: hanging", 8.0, "Let the arm hang naturally and do not move at all. (task step 1)"),
    ("hold_forward", "Still: forward, level", 8.0, "Hold the arm straight forward, level with the floor, and keep still. (step 2)"),
    ("hold_left_open", "Still: front-left, hand open", 8.0, "Swing the arm front-left to where you would grab the tape, hand open, and keep still. (step 3)"),
    ("hold_left_grip", "Still: front-left, gripping", 8.0, "Same position, make a fist (as if holding the tape) and keep still. (step 4)"),
    ("hold_lifted_grip", "Still: lifted, gripping", 8.0, "Keep the fist, raise the arm a little, and stop. (step 5)"),
    ("hold_right_grip", "Still: right, gripping", 8.0, "Keep the fist, swing the arm to the right, and stop. (step 6)"),
    ("hold_place_grip", "Still: place position, gripping", 8.0, "Keep the fist, lower the arm to where the tape goes, and stop. (step 7)"),
    ("aim_left", "Slow fine-tuning: aiming at the tape", 15.0, "Near the front-left grab position, make slow, small corrections left/right/up/down, as if aiming."),
    ("task_1", "Full task, run 1", 20.0, "Do all 7 steps at demo speed: hang -> forward -> swing left -> grip -> lift -> swing right -> place."),
    ("task_2", "Full task, run 2", 20.0, "Do the full task again."),
    ("task_3", "Full task, run 3", 20.0, "Do the full task again."),
    ("check_hold_right_forward", "Check: still, front-right", 8.0, "(not in the task) Reach front-right, hand open, and keep still."),
    ("check_hold_high", "Check: still, arm raised high", 8.0, "(not in the task) Raise the arm high in front and keep still."),
    ("check_free", "Check: free movement", 15.0, "(not in the task) Move freely, fast and slow, in every direction."),
]
CHECK_PREFIX = "check_"   # phases kept out of the tuning, used only to check the chosen parameters generalise

# 2026-10-04: how much does the base move when the arm is only RAISED (the natural upper-arm twist, which differs by
# direction) versus an intended swing? Decides the base's "slow follow while raising" from raw data, not a guess.
# A lower-left -> upper-right raise moves up and sideways at once: kept out as a check.
BASE_RAISE_PHASES = [
    ("raise_forward", "Raise forward and lower", 12.0, "From hanging, raise straight forward to horizontal and lower again, twice. Only raise: no deliberate twist or sideways swing."),
    ("raise_left_front", "Raise front-left and lower", 12.0, "From hanging, raise front-left (toward the tape) to horizontal and lower again, twice. Only raise: no deliberate twist."),
    ("raise_right_front", "Raise front-right and lower", 12.0, "From hanging, raise front-right (toward where the tape goes) to horizontal and lower again, twice. Only raise: no deliberate twist."),
    ("raise_slow", "Raise forward slowly and lower", 25.0, "From hanging, raise straight forward slowly, taking about 10 s to reach horizontal, then lower slowly. Only raise: no deliberate twist."),
    ("swing_only", "Level arm, sideways swing only", 12.0, "Arm forward and level; at the same height swing left -> right -> center, twice. Do not raise or lower deliberately."),
    ("hold_forward", "Still: forward, level", 8.0, "Hold the arm straight forward, level with the floor, and keep still."),
    ("check_diagonal", "Check: diagonal, lower-left to upper-right", 12.0, "(not in the task) Raise diagonally from lower-left to upper-right and back, twice."),
]
PHASE_SETS = {"filter": PHASES, "base_raise": BASE_RAISE_PHASES}
DEFAULT_SET = "filter"

SPEED_CUTOFF_HZ = 1.0     # the 1-euro filter's own derivative low-pass (d_cutoff, the paper's recommended 1 Hz)


def parse_tick(text):
    """(upper_arm, forearm) raw accel vectors (g) from one firmware tick line, each None if absent. Parsed by name: the
    real firmware sends elbow_raw BEFORE shoulder_raw."""
    out = []
    for fw in ("shoulder", "elbow"):
        m = re.search(rf"{fw}_raw_ax=(\S+) {fw}_raw_ay=(\S+) {fw}_raw_az=(\S+)", text)
        out.append(tuple(float(x) for x in m.groups()) if m else None)
    return out[0], out[1]


def record_phase(read_line, clock, seconds, on_second=None):
    """[(t, upper, fore), ...] for `seconds`, t from the phase start. Lines without sensor data are skipped."""
    start = clock()
    samples = []
    next_tick = 1.0
    while True:
        text = read_line()
        t = clock() - start
        if t > seconds:
            break
        up, fore = parse_tick(text)
        if up is not None or fore is not None:
            samples.append((t, up, fore))
        if on_second and t >= next_tick:
            on_second(int(next_tick), len(samples))
            next_tick += 1.0
    return samples


def _vectors(samples, sensor):
    idx = 1 if sensor == "upper_arm" else 2
    return [(s[0], s[idx]) for s in samples if s[idx] is not None]


def speeds(samples, sensor, cutoff_hz=SPEED_CUTOFF_HZ):
    """The speed the 1-euro filter sees: |d(raw accel vector)/dt| low-passed at cutoff_hz, in g/s -- the RAW vector, not
    normalised, exactly like Vec3OneEuro, so the parameters chosen from it mean the same thing on the chip. With
    |a| ~ 1 g (a still or slowly moving arm) it is about the arm's rotation rate in rad/s."""
    vs = _vectors(samples, sensor)
    out = []
    vel = (0.0, 0.0, 0.0)
    for (t0, a), (t1, b) in zip(vs, vs[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        d = tuple((cb - ca) / dt for ca, cb in zip(a, b))
        alpha = 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff_hz * dt))
        vel = tuple(v + alpha * (x - v) for v, x in zip(vel, d))
        out.append(math.sqrt(sum(v * v for v in vel)))
    return out


def speed_stats(samples, sensor):
    sp = sorted(speeds(samples, sensor)[20:])          # skip the low-pass's own start-up
    if not sp:
        return {"median": None, "p90": None, "max": None, "n": 0}
    pick = lambda q: sp[min(len(sp) - 1, int(q * len(sp)))]
    return {"median": pick(0.5), "p90": pick(0.9), "max": sp[-1], "n": len(sp)}


def noise_std(samples, sensor):
    """Mean per-axis standard deviation of the raw accel (g) -- meaningful for the rest phase."""
    vs = [v for _t, v in _vectors(samples, sensor)]
    if len(vs) < 2:
        return None
    stds = []
    for axis in range(3):
        xs = [v[axis] for v in vs]
        m = sum(xs) / len(xs)
        stds.append(math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1)))
    return sum(stds) / 3.0


def save(path, phases, port):
    path = Path(path)
    if path.exists():
        raise FileExistsError(f"{path} already exists and will not be overwritten")
    payload = {
        "captured_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "port": port,
        "note": "raw accel (g) from phase3_control_loop tick lines; t in s from each phase's start",
        "phases": {name: [{"t": t, "upper_arm": list(u) if u else None, "forearm": list(f) if f else None}
                          for t, u, f in samples]
                   for name, samples in phases.items()},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=1) + "\n")


def summary_lines(phases):
    """Per phase and sensor: how fast the arm moved (g/s, about rad/s) and, for still holds, the resting noise (g)."""
    fmt = lambda v: "-" if v is None else f"{v:.3f}"
    lines = []
    for name, samples in phases.items():
        label = next((p[1] for ps in PHASE_SETS.values() for p in ps if p[0] == name), name)
        tag = " (check only, not used for tuning)" if name.startswith(CHECK_PREFIX) else ""
        lines.append(f"\n[{label}]{tag}")
        for sensor, (slabel, addr, _fw) in SENSORS.items():
            st = speed_stats(samples, sensor)
            row = (f"  {slabel} {addr}: {st['n']} samples, speed median {fmt(st['median'])} / 90% {fmt(st['p90'])} / "
                   f"max {fmt(st['max'])} g/s")
            if name.startswith(("hold_", "check_hold_")):
                row += f", resting noise {fmt(noise_std(samples, sensor))} g"
            lines.append(row)
    return lines


def wait_for_data(read_line, clock, timeout_s=3.0):
    start = clock()
    while clock() - start < timeout_s:
        up, fore = parse_tick(read_line())
        if up is not None and fore is not None:
            return True
    print("No sensor data received. Is the board running phase3_control_loop, and did the app start after flashing? "
          "(check with python3 tools/check_hardware_ready.py --boot-check)")
    return False


def run_session(read_line, clock, input_fn=input, phases=PHASES, countdown_s=3.0):
    recorded = {}
    for i, (name, label, seconds, how) in enumerate(phases, 1):
        while True:
            print(f"\n=== phase {i}/{len(phases)}: {label} ({seconds:.0f} s) ===")
            print(f"   {how}")
            input_fn("   Get into position, then press Enter to record: ")
            end = clock() + countdown_s
            last_shown = None
            while clock() < end:                       # keep reading so the recording starts from fresh data
                read_line()
                left = math.ceil(end - clock())
                if left != last_shown and left > 0:
                    print(f"   {left}…", flush=True)
                    last_shown = left
            print("   ● recording", flush=True)
            samples = record_phase(read_line, clock, seconds,
                                   on_second=lambda s, n: print(f"\r   ● recording {s:2d}/{seconds:.0f} s ({n} samples)",
                                                                end="", flush=True))
            st = speed_stats(samples, "upper_arm")
            med = "-" if st["median"] is None else f"{st['median']:.2f} g/s (about rad/s)"
            print(f"\n   Done: {len(samples)} samples, upper-arm median speed {med}")
            if input_fn("   Keep this phase? Enter=keep  r=re-record: ").strip().lower() == "r":
                print("   Re-recording this phase.")
                continue
            recorded[name] = samples
            break
    return recorded


def main():
    import serial
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from usb_serial_port import autodetect_port
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    parser.add_argument("--set", default=DEFAULT_SET, choices=sorted(PHASE_SETS),
                        help="filter = the 1-euro tuning set (default); base_raise = raising vs swinging, for the base")
    args = parser.parse_args()
    phase_list = PHASE_SETS[args.set]
    port = args.port or autodetect_port()
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open the serial port {port}: {exc}\nMost common cause: run_demo_live.py or watch_imu_raw.py is still running; close it and retry.")
    read_line = lambda: ser.readline().decode(errors="ignore")
    try:
        print(f"Reading {port} (nothing is flashed, read only). {len(phase_list)} phases, each can be re-recorded. Ctrl+C stops at any time.")
        if not wait_for_data(read_line, time.monotonic):
            sys.exit(1)
        phases = run_session(read_line, time.monotonic, phases=phase_list)
    except KeyboardInterrupt:
        sys.exit("\nInterrupted; nothing saved.")
    finally:
        ser.close()
    out = DATA_DIR / f"arm_motion_{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
    save(out, phases, port)
    print("\n".join(summary_lines(phases)))
    print(f"\nSaved to {out}\nUse this file to choose the filter parameters.")


if __name__ == "__main__":
    main()

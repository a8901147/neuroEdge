#!/usr/bin/env python3
"""Measure how the real MEArm's link angles depend on the servo pulses (Path B's missing last step).

Path B produces MuJoCo model commands; the real arm takes servo pulse widths. Both of the real arm's servos sit at the base
and drive their links through parallel linkages, so the UPPER ARM's absolute elevation depends on the shoulder pulse alone
and the FOREARM's on the elbow pulse alone (mujoco_bridge/mearm_pulse_map.py). This tool moves ONE servo at a time, in
small paced steps and only inside the measured safe envelope, and asks you to type the link's angle each time -- a number
read off a phone inclinometer, not a judgement by eye.

How to read an angle: lay the phone flat against the side of the link with its long edge along the link (the line between the
two pivot screws), the top edge pointing toward the link's FAR end (upper arm: toward the elbow; forearm: toward the claw).
Each reading is two questions: the number the app shows (no minus sign), then whether the link's far end is higher or
lower than its near end (h / l) -- the sign comes only from what you SEE, because phone apps show no sign and the forearm
crosses horizontal within its range. Type q to stop; what was measured is kept.

Needs `servo_limit_finder_4ch` flashed + power-cycled (same protocol as servo_pose_4ch.py). Each run writes a NEW file
data/mearm_angles_<timestamp>.json, never overwriting one.

    python3 tools/measure_servo_angles.py
    python3 tools/measure_servo_angles.py --analyze data/mearm_angles_<timestamp>.json     # no hardware
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "mujoco_bridge"))
import gen_mearm_envelope as env  # noqa: E402
import measure_linkage_region as mlr  # noqa: E402
import mearm_pathb as pb  # noqa: E402
import mearm_pulse_map as pm  # noqa: E402
import servo_pose_4ch as sp  # noqa: E402

SHOULDER, ELBOW = 2, 3
# Part A: shoulder through its measured range and back (a repeat at rest shows backlash); elbow stays at rest.
SHOULDER_POINTS = (1500, 1575, 1650, 1725, 1800, 1650, 1500)
SHOULDER_ELBOW_US = 1500
# Part B: elbow through its whole range and back, with the shoulder where the elbow's safe window is widest.
ELBOW_POINTS = (1500, 1700, 1850, 1300, 1000, 700, 500, 1000, 1500)
ELBOW_SHOULDER_US = 1575
MAX_ABS_ANGLE_DEG = 180.0        # any elevation is physically possible (a link can lean back past vertical); this only catches typos like 400
RESIDUAL_WARN_DEG = 3.0
DATA_DIR = HERE.parent / "data"


def default_out_path(now=None):
    return DATA_DIR / time.strftime("mearm_angles_%Y%m%d-%H%M%S.json", now or time.localtime())


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


def ask_signed_angle(prompt, far_end, input_fn=None, out=print):
    """The link's elevation in degrees, or None if the person typed q.

    Two questions, because a phone inclinometer app shows an angle WITHOUT a sign and the forearm crosses horizontal within
    its range (the first real run had two readings with the wrong sign: fit residual 23 deg): first the number as the app
    shows it, then -- what the person SEES -- whether the far end is higher or lower than the near end. The sign comes only
    from that second answer; a typed minus sign is refused. (`input` is looked up at CALL time: a default argument would bind
    the built-in at import and a patched input would never be used.)"""
    ask = input_fn or input
    while True:
        answer = ask(prompt).strip().lower()
        if answer == "q":
            return None
        try:
            value = float(answer)
        except ValueError:
            value = math.nan
        if math.isfinite(value) and value < 0:
            out("  No minus sign needed: type the number the phone shows; the next question asks higher or lower.")
            continue
        if math.isfinite(value) and value <= MAX_ABS_ANGLE_DEG:
            break
        out(f"  unrecognized. Type a number from 0 to {MAX_ABS_ANGLE_DEG:.0f} (degrees), or q to quit.")
    if value == 0.0:
        return 0.0                                                       # horizontal: there is no sign to give
    while True:
        answer = ask(f"  Is the {far_end} end of this link higher or lower than the other end? "
                     f"(h=higher, l=lower, q to quit): ").strip().lower()
        if answer == "q":
            return None
        if answer in ("h", "l"):
            return value if answer == "h" else -value
        out("  unrecognized. Type h (higher) or l (lower).")


def _prompt(link):
    return f"  {link}: elevation in degrees? (the number the phone shows, no minus sign, horizontal = 0; q to quit): "


def measure(board, table, out_path, log=print):
    """Both parts of the protocol. Returns the data dict (also saved after every reading)."""
    mlr.check_output_path(out_path, overwrite=False)                    # before the arm is touched
    data = {"measured_at": time.strftime("%Y-%m-%d %H:%M:%S"), "step_us": sp.STEP_US,
            "shoulder_points": [], "elbow_points": [], "envelope_shoulders": table["shoulders"]}

    def go(ch, pulse, other_ch):
        """Move ONE servo, after checking the pose it would create -- with the OTHER servo where it REALLY is (read
        back from the board), not where the plan meant it to be."""
        other = sp.read_state(board, other_ch)
        want = (pulse, other) if ch == SHOULDER else (other, pulse)
        if env.clamp(table, *want) != want:
            raise ValueError(f"planned pose shoulder={want[0]} elbow={want[1]} is outside the safe envelope")
        mlr.walk(board, ch, pulse, None)

    stopped = False
    try:
        mlr.return_to_rest(board)
        go(ELBOW, SHOULDER_ELBOW_US, SHOULDER)
        log(f"\n=== Part 1: shoulder -> upper-arm angle (elbow held at {SHOULDER_ELBOW_US}) ===")
        for pulse in SHOULDER_POINTS:
            go(SHOULDER, pulse, ELBOW)
            actual = sp.read_state(board, SHOULDER)
            elbow_now = sp.read_state(board, ELBOW)
            log(f"\nShoulder now at {actual} µs.")
            angle = ask_signed_angle(_prompt("Upper arm (the link from the shoulder pivot to the elbow pivot)"), "elbow")
            if angle is None:
                stopped = True
                break
            data["shoulder_points"].append({"pulse": actual, "angle_deg": angle, "typed_deg": abs(angle),
                                            "elbow_pulse": elbow_now})
            save_json(out_path, data)
        if not stopped:
            mlr.return_to_rest(board)
            go(SHOULDER, ELBOW_SHOULDER_US, ELBOW)
            log(f"\n=== Part 2: elbow -> forearm angle (shoulder held at {ELBOW_SHOULDER_US}) ===")
            for pulse in ELBOW_POINTS:
                go(ELBOW, pulse, SHOULDER)
                actual = sp.read_state(board, ELBOW)
                shoulder_now = sp.read_state(board, SHOULDER)
                log(f"\nElbow now at {actual} µs.")
                angle = ask_signed_angle(_prompt("Forearm (the link from the elbow pivot to the claw)"), "claw")
                if angle is None:
                    break
                data["elbow_points"].append({"pulse": actual, "angle_deg": angle, "typed_deg": abs(angle),
                                             "shoulder_pulse": shoulder_now})
                save_json(out_path, data)
    finally:
        mlr.safe_finish(board, log=log)
    return data


def _fit(points):
    return pm.fit_angle_vs_pulse([(p["pulse"], p["angle_deg"]) for p in points])


def fit_measurements(data):
    return {"shoulder": _fit(data["shoulder_points"]), "elbow": _fit(data["elbow_points"])}


def hysteresis(data):
    """|first reading - last reading| at the 1500 rest, per part: how far the angle differs after going out and back."""
    def gap(points):
        return abs(points[-1]["angle_deg"] - points[0]["angle_deg"]) if len(points) >= 2 else math.nan
    return {"shoulder": gap(data["shoulder_points"]), "elbow": gap(data["elbow_points"])}


def report(data, out=print):
    fits = {}
    for name, key in (("shoulder -> upper arm", "shoulder_points"), ("elbow -> forearm", "elbow_points")):
        try:
            f = _fit(data[key])
        except ValueError as exc:
            out(f"{name}: not enough usable readings ({exc}; needs at least 3, not all at the same pulse width)")
            continue
        fits[key] = f
        out(f"{name}: elevation = {f['at_1500']:+.1f}° + {f['slope']:+.4f}°/µs × (pulse − 1500)   "
            f"({f['n']} readings, max residual {f['max_residual']:.1f}°)")
        if not pm.slope_is_plausible(f["slope"]):
            out(f"  ⚠ slope {f['slope']:+.4f}°/µs is implausible (an SG92R is about 0.09°/µs; the linkage can scale that, "
                f"but a tenfold difference is usually a unit or typing error)")
        if f["max_residual"] > RESIDUAL_WARN_DEG:
            out(f"  ⚠ max residual {f['max_residual']:.1f}° exceeds {RESIDUAL_WARN_DEG:.0f}°: a reading may be mistyped, "
                f"or the link is not linear")
    h = hysteresis(data)
    for name, key in (("shoulder", "shoulder"), ("elbow", "elbow")):
        if not math.isnan(h[key]):
            out(f"{name}: after a round trip, the angle at the same pulse (1500) differs by {h[key]:.1f}° (backlash)")
    if "shoulder_points" in fits:
        fs = fits["shoulder_points"]
        fe = fits.get("elbow_points", fs)
        try:
            m = pm.ServoAngleMap(shoulder=fs, elbow=fe)
        except ValueError:
            return
        table_range = (1500, 1800)
        model_range = (pb.SHOULDER_RAISED, pb.SHOULDER_REST)
        share = m.shoulder_travel_share(table_range, model_range)
        lo, hi = m.reachable_shoulder_ctrl(table_range)
        out(f"\nShoulder pulses {table_range[0]}-{table_range[1]} (the measured safe range) map to model shoulder commands "
            f"{lo:+.3f} to {hi:+.3f} rad, about {share * 100:.0f}% of the model's shoulder travel "
            f"({model_range[0]:+.3f} to {model_range[1]:+.3f}).")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    parser.add_argument("--cp2102", action="store_true")
    parser.add_argument("--out", type=Path, default=None, help="result file (default: a NEW timestamped one)")
    parser.add_argument("--analyze", type=Path, metavar="FILE", help="no hardware: fit and report a result file")
    args = parser.parse_args()
    if args.analyze:
        data = json.loads(args.analyze.read_text())
        if not isinstance(data, dict) or "shoulder_points" not in data:
            sys.exit(f"{args.analyze} is not an angle measurement file")
        report(data)
        return
    out_path = args.out or default_out_path()
    try:
        mlr.check_output_path(out_path, overwrite=False)
    except FileExistsError as exc:
        sys.exit(str(exc))
    import serial
    from usb_serial_port import autodetect_port
    table = env.build_table(mlr.load_records([env.ROOT / "data" / n for n in env.SOURCES]))
    board = serial.Serial(args.port or autodetect_port(prefer_cp2102=args.cp2102), sp.BAUD, timeout=0.5)
    print("Current pulse widths: " + "  ".join(f"{sp.NAMES[ch - 1]}={sp.read_state(board, ch)}" for ch in sp.RANGES))
    print(f"\nResults go to: {out_path}\n")
    print("How to measure: hold the phone flat against the side of the link, long edge along the link (the line "
          "between its two pivot screws).\nEach position asks two questions: (1) the angle the phone shows (no minus "
          "sign); (2) whether the link's far end (upper arm: the elbow end; forearm: the claw end) is higher or lower "
          "than the other end. Only your eyes decide the sign, not the phone. q quits; what was measured is kept.")
    print("The arm moves one servo at a time, slowly, step by step, only inside the measured safe envelope. Clear the "
          "space around the arm.")
    input("Press Enter to start: ")
    data = measure(board, table, out_path)
    print(f"\nRaw results saved in {out_path}\n")
    report(data)


if __name__ == "__main__":
    main()

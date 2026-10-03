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
            out("  不用打負號:先輸入手機顯示的數字,下一題我會問高還是低。")
            continue
        if math.isfinite(value) and value <= MAX_ABS_ANGLE_DEG:
            break
        out(f"  看不懂。請輸入 0 到 {MAX_ABS_ANGLE_DEG:.0f} 之間的數字(度),或 q 結束。")
    if value == 0.0:
        return 0.0                                                       # horizontal: there is no sign to give
    while True:
        answer = ask(f"  這根連桿的{far_end}端,比另一端 高還是低?(h=高, l=低, q 結束): ").strip().lower()
        if answer == "q":
            return None
        if answer in ("h", "l"):
            return value if answer == "h" else -value
        out("  看不懂。請輸入 h(高)或 l(低)。")


def _prompt(link):
    return f"  {link}的仰角是幾度?(手機顯示的數字,不用打負號,水平=0;q 結束): "


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
        log(f"\n=== 第一部分:肩膀 → 上臂角度(手肘固定在 {SHOULDER_ELBOW_US})===")
        for pulse in SHOULDER_POINTS:
            go(SHOULDER, pulse, ELBOW)
            actual = sp.read_state(board, SHOULDER)
            elbow_now = sp.read_state(board, ELBOW)
            log(f"\n肩膀現在在 {actual} µs。")
            angle = ask_signed_angle(_prompt("上臂(肩膀轉軸到手肘轉軸那根連桿)"), "手肘")
            if angle is None:
                stopped = True
                break
            data["shoulder_points"].append({"pulse": actual, "angle_deg": angle, "typed_deg": abs(angle),
                                            "elbow_pulse": elbow_now})
            save_json(out_path, data)
        if not stopped:
            mlr.return_to_rest(board)
            go(SHOULDER, ELBOW_SHOULDER_US, ELBOW)
            log(f"\n=== 第二部分:手肘 → 前臂角度(肩膀固定在 {ELBOW_SHOULDER_US})===")
            for pulse in ELBOW_POINTS:
                go(ELBOW, pulse, SHOULDER)
                actual = sp.read_state(board, ELBOW)
                shoulder_now = sp.read_state(board, SHOULDER)
                log(f"\n手肘現在在 {actual} µs。")
                angle = ask_signed_angle(_prompt("前臂(手肘轉軸到夾爪那根連桿)"), "夾爪")
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
    for name, key in (("肩膀→上臂", "shoulder_points"), ("手肘→前臂", "elbow_points")):
        try:
            f = _fit(data[key])
        except ValueError as exc:
            out(f"{name}: 讀數不夠或不能用({exc};至少要 3 筆、且不能全在同一個脈寬)")
            continue
        fits[key] = f
        out(f"{name}: 仰角 = {f['at_1500']:+.1f}° + {f['slope']:+.4f}°/µs × (脈寬 − 1500)   "
            f"({f['n']} 筆,最大誤差 {f['max_residual']:.1f}°)")
        if not pm.slope_is_plausible(f["slope"]):
            out(f"  ⚠ 斜率 {f['slope']:+.4f}°/µs 不合理(SG92R 約 0.09°/µs,連桿可放大或縮小,但差十倍多半是單位或打字錯誤)")
        if f["max_residual"] > RESIDUAL_WARN_DEG:
            out(f"  ⚠ 最大誤差 {f['max_residual']:.1f}° 超過 {RESIDUAL_WARN_DEG:.0f}°:有一筆讀數可能打錯、或連桿不是直線關係")
    h = hysteresis(data)
    for name, key in (("肩膀", "shoulder"), ("手肘", "elbow")):
        if not math.isnan(h[key]):
            out(f"{name}來回一趟後,同一個脈寬(1500)的角度差 {h[key]:.1f}°(間隙/回彈)")
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
        out(f"\n肩膀脈寬 {table_range[0]}~{table_range[1]}(量過的安全範圍)對應到模型肩膀指令 {lo:+.3f}~{hi:+.3f} rad,"
            f"約佔模型肩膀行程({model_range[0]:+.3f}~{model_range[1]:+.3f})的 {share * 100:.0f}%。")


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
            sys.exit(f"{args.analyze} 不是角度量測檔")
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
    print("目前脈寬: " + "  ".join(f"{sp.NAMES[ch - 1]}={sp.read_state(board, ch)}" for ch in sp.RANGES))
    print(f"\n結果存到: {out_path}\n")
    print("怎麼量角度:把手機平貼在連桿側面,長邊沿著連桿(兩個轉軸螺絲的連線)。\n"
          "每一個位置會問兩題:(1) 手機顯示的角度數字(不用打負號);(2) 這根連桿的遠端(上臂:手肘那頭;前臂:夾爪那頭)"
          "比另一端『高還是低』——正負號只由你眼睛看到的這一題決定,不由手機決定。q 結束,已量的會保留。")
    print("手臂會一次一顆、一步一步慢慢移動,只在量過的安全包絡內。請清空手臂周圍。")
    input("準備好就按 Enter 開始: ")
    data = measure(board, table, out_path)
    print(f"\n原始結果存在 {out_path}\n")
    report(data)


if __name__ == "__main__":
    main()

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
    python3 tools/capture_arm_motion.py --set emg           # ~1.5 min: grip misfires / releases while the arm moves
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
SENSORS = {"upper_arm": ("上臂", "0x68", "shoulder"), "forearm": ("前臂", "0x69", "elbow")}
# Built around the core demo task (hang -> forward -> left -> grip tape -> lift -> right -> place; SESSION_LOG
# 2026-10-02): still holds at each task pose (jitter depends on the pose -- hanging is where the base direction is least
# stable; gripping adds the ~9-10 Hz physiological tremor measured 2026-09-12), slow fine aiming, the whole task at demo
# speed, and finally poses/motions that are NOT in the task, kept apart as a check against tuning only for the task.
PHASES = [
    ("hold_hang", "靜止：垂下", 8.0, "手臂自然垂下，完全不要動。（任務第 1 步）"),
    ("hold_forward", "靜止：往前伸平", 8.0, "手臂往前伸直、和地面平行，停住不動。（第 2 步）"),
    ("hold_left_open", "靜止：左前方、手張開", 8.0, "手臂往左前方擺，停在要抓膠帶的位置，手張開，不要動。（第 3 步）"),
    ("hold_left_grip", "靜止：左前方、握拳", 8.0, "同一個位置，握拳（像抓住膠帶），維持握著不動。（第 4 步）"),
    ("hold_lifted_grip", "靜止：抬起、握拳", 8.0, "握著拳，手臂往上抬一點，停住。（第 5 步）"),
    ("hold_right_grip", "靜止：右方、握拳", 8.0, "握著拳，手臂擺到右方，停住。（第 6 步）"),
    ("hold_place_grip", "靜止：放下的位置、握拳", 8.0, "握著拳，手臂往下到要放膠帶的位置，停住。（第 7 步）"),
    ("aim_left", "慢慢微調：對準膠帶", 15.0, "在左前方抓膠帶的位置附近，慢慢、小幅度地左右上下修正，像在對準。"),
    ("task_1", "完整任務 第 1 次", 20.0, "用 demo 的速度把 7 步做完：垂下→往前伸平→左擺→握拳→抬起→右擺→放下。"),
    ("task_2", "完整任務 第 2 次", 20.0, "再做一次完整任務。"),
    ("task_3", "完整任務 第 3 次", 20.0, "再做一次完整任務。"),
    ("check_hold_right_forward", "驗證：右前方靜止", 8.0, "（不在任務裡）手臂往右前方伸，手張開，停住不動。"),
    ("check_hold_high", "驗證：手舉高靜止", 8.0, "（不在任務裡）手臂往前上方舉高，停住不動。"),
    ("check_free", "驗證：自由動作", 15.0, "（不在任務裡）隨意地動，快慢都有、各個方向都有。"),
]
CHECK_PREFIX = "check_"   # phases kept out of the tuning, used only to check the chosen parameters generalise

# 2026-10-04: how much does the base move when the arm is only RAISED (the natural upper-arm twist, which differs by
# direction) versus an intended swing? Decides the base's "slow follow while raising" from raw data, not a guess.
# The user's lower-left -> upper-right case raises and swings at once: kept out as a check.
BASE_RAISE_PHASES = [
    ("raise_forward", "往前舉起放下", 12.0, "從垂下往正前方舉到水平再放下，做 2 次。只舉，不要刻意轉或左右擺。"),
    ("raise_left_front", "往左前方舉起放下", 12.0, "從垂下往左前方（抓膠帶的方向）舉到水平再放下，做 2 次。只舉，不要刻意轉。"),
    ("raise_right_front", "往右前方舉起放下", 12.0, "從垂下往右前方（放膠帶的方向）舉到水平再放下，做 2 次。只舉，不要刻意轉。"),
    ("raise_slow", "慢慢往前舉起放下", 25.0, "從垂下往正前方，用大約 10 秒慢慢舉到水平，再慢慢放下。只舉，不要刻意轉。"),
    ("swing_only", "伸平後只左右擺", 12.0, "手臂往前伸平，在同一個高度左擺→右擺→回正，做 2 次。不要刻意舉高或放低。"),
    ("hold_forward", "靜止：往前伸平", 8.0, "手臂往前伸直、和地面平行，停住不動。"),
    ("check_diagonal", "驗證：左下舉到右上", 12.0, "（不在任務裡）從左下方斜斜舉到右上方再回來，做 2 次。"),
]
# 2026-10-04: the EMG calibration logs only hold a STILL arm (relaxed / clenched ~2 s each). Whether the grip
# misfires while the relaxed arm moves, or lets go while a light grip is carried through the demo, needs EMG recorded
# while the arm moves -- the tick line already carries each 10 ms window's emg_min/emg_max.
EMG_PHASES = [
    ("relaxed_still", "手放鬆、手臂垂下不動", 8.0, "手完全放鬆（不要握拳），手臂自然垂下，不要動。"),
    ("relaxed_task", "手放鬆，手臂照 demo 動", 20.0,
     "手保持放鬆、不要握拳，手臂照 demo 的路線動：垂下→往前伸平→左擺→抬起→右擺→放下。（看會不會誤觸）"),
    ("grip_still", "輕輕握拳、手臂不動", 8.0, "像拿著膠帶那樣輕輕握拳（不用太用力），手臂停在左前方不動。"),
    ("grip_task", "輕輕握拳，手臂照 demo 動", 20.0,
     "保持輕輕握拳不放開，手臂照 demo 第 5–7 步動：抬起→右擺→放下，可以重複。（看會不會誤放開）"),
    ("grip_firm_still", "用力握拳、手臂不動", 5.0, "用力握拳（最大力氣的七八成），手臂不動。"),
    ("check_open_close", "驗證：握拳、放開重複", 15.0, "（不在任務裡）握拳約 1 秒、放開約 1 秒，重複 5 次。"),
]
PHASE_SETS = {"filter": PHASES, "base_raise": BASE_RAISE_PHASES, "emg": EMG_PHASES}
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


def parse_emg(text):
    """(emg_min, emg_max) of the firmware's 10 ms window from one tick line (raw 12-bit ADC counts), or None."""
    m = re.search(r"emg_min=(\d+) emg_max=(\d+)", text)
    return (int(m.group(1)), int(m.group(2))) if m else None


def record_phase(read_line, clock, seconds, on_second=None):
    """[(t, upper, fore, emg), ...] for `seconds`, t from the phase start; emg = (min, max) or None. Lines without
    sensor data are skipped."""
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
            samples.append((t, up, fore, parse_emg(text)))
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
        raise FileExistsError(f"{path} 已經存在，不會覆蓋")
    payload = {
        "captured_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "port": port,
        "note": "raw accel (g) and the 10 ms EMG window [emg_min, emg_max] (ADC counts) from phase3_control_loop tick "
                "lines; t in s from each phase's start",
        "phases": {name: [{"t": s[0], "upper_arm": list(s[1]) if s[1] else None,
                           "forearm": list(s[2]) if s[2] else None,
                           "emg": list(s[3]) if len(s) > 3 and s[3] else None}
                          for s in samples]
                   for name, samples in phases.items()},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=1) + "\n")


GRIP_ON_S = GRIP_OFF_S = 0.15   # firmware kOnDuration / kOffDuration


def emg_events(samples, threshold, release, on_s=GRIP_ON_S, off_s=GRIP_OFF_S):
    """The grip decision replayed like GripStateMachine, starting released: grip once the value stays ABOVE `threshold`
    for on_s; while gripping, release once it stays BELOW `release` for off_s. The firmware compares a 1 kHz EMA of
    the envelope; the tick line only has each 10 ms window's min/max, so their midpoint stands in for it."""
    vals = [((s[3][0] + s[3][1]) / 2.0, s[0]) for s in samples if len(s) > 3 and s[3]]
    gripping, above, below, grips, releases = False, 0.0, 0.0, 0, 0
    n_above = n_below = 0
    prev_t = None
    for v, t in vals:
        dt = 0.0 if prev_t is None else t - prev_t
        prev_t = t
        n_above += v > threshold
        n_below += v < release
        level = release if gripping else threshold
        if v > level:
            above, below = above + dt, 0.0
        else:
            below, above = below + dt, 0.0
        if not gripping and above >= on_s:
            gripping, grips, above = True, grips + 1, 0.0
        elif gripping and below >= off_s:
            gripping, releases, below = False, releases + 1, 0.0
    n = len(vals)
    return {"n": n, "grips": grips, "releases": releases,
            "above_grip": n_above / n if n else 0.0, "below_release": n_below / n if n else 0.0,
            "min": min(v for v, _ in vals) if vals else None, "max": max(v for v, _ in vals) if vals else None,
            "median": sorted(v for v, _ in vals)[n // 2] if vals else None}


def summary_lines(phases, emg_thresholds=None):
    """Per phase and sensor: how fast the arm moved (g/s, about rad/s) and, for still holds, the resting noise (g)."""
    fmt = lambda v: "-" if v is None else f"{v:.3f}"
    lines = []
    for name, samples in phases.items():
        label = next((p[1] for ps in PHASE_SETS.values() for p in ps if p[0] == name), name)
        tag = "（驗證用，不參與調參）" if name.startswith(CHECK_PREFIX) else ""
        lines.append(f"\n【{label}】{tag}")
        for sensor, (slabel, addr, _fw) in SENSORS.items():
            st = speed_stats(samples, sensor)
            row = (f"  {slabel}{addr}：{st['n']} 筆，速度 中位數 {fmt(st['median'])} / 90% {fmt(st['p90'])} / "
                   f"最大 {fmt(st['max'])} g/s")
            if name.startswith(("hold_", "check_hold_")):
                row += f"，靜止雜訊 {fmt(noise_std(samples, sensor))} g"
            lines.append(row)
        if any(len(s) > 3 and s[3] for s in samples):
            th, rel = emg_thresholds if emg_thresholds else (float("inf"), float("-inf"))
            ev = emg_events(samples, th, rel)
            row = f"  EMG：{ev['n']} 筆，最低 {ev['min']:.0f} / 中位數 {ev['median']:.0f} / 最高 {ev['max']:.0f}"
            if emg_thresholds:
                row += (f"；高於抓握門檻 {th:.0f} 的時間 {ev['above_grip']:.0%}，低於放開門檻 {rel:.0f} 的時間 "
                        f"{ev['below_release']:.0%}")
                if name.startswith("relaxed"):
                    row += f"；→ 誤觸（手放鬆卻判成抓握）{ev['grips']} 次"
                elif name.startswith("grip"):
                    row += f"；→ 抓到 {ev['grips']} 次，誤放開（還握著卻判成放開）{ev['releases']} 次"
            lines.append(row)
    return lines


def wait_for_data(read_line, clock, timeout_s=3.0):
    start = clock()
    while clock() - start < timeout_s:
        up, fore = parse_tick(read_line())
        if up is not None and fore is not None:
            return True
    print("沒有收到感測器資料——板子上跑的是 phase3_control_loop 嗎？燒錄後有沒有 reset run？"
          "（可用 python3 tools/check_hardware_ready.py --boot-check 確認）")
    return False


def run_session(read_line, clock, input_fn=input, phases=PHASES, countdown_s=3.0):
    recorded = {}
    for i, (name, label, seconds, how) in enumerate(phases, 1):
        while True:
            print(f"\n=== 第 {i}/{len(phases)} 段：{label}（{seconds:.0f} 秒）===")
            print(f"   {how}")
            input_fn("   擺好姿勢後按 Enter 開始錄：")
            end = clock() + countdown_s
            last_shown = None
            while clock() < end:                       # keep reading so the recording starts from fresh data
                read_line()
                left = math.ceil(end - clock())
                if left != last_shown and left > 0:
                    print(f"   {left}…", flush=True)
                    last_shown = left
            print("   ● 錄製中", flush=True)
            samples = record_phase(read_line, clock, seconds,
                                   on_second=lambda s, n: print(f"\r   ● 錄製中 {s:2d}/{seconds:.0f} 秒（{n} 筆）",
                                                                end="", flush=True))
            st = speed_stats(samples, "upper_arm")
            med = "-" if st["median"] is None else f"{st['median']:.2f} g/s（約 rad/s）"
            print(f"\n   完成：{len(samples)} 筆，上臂速度中位數 {med}")
            if input_fn("   保留這段嗎？ Enter=保留  r=重錄：").strip().lower() == "r":
                print("   重錄這一段。")
                continue
            recorded[name] = samples
            break
    return recorded


CALIBRATION_FILE = Path(__file__).resolve().parent / "mujoco_bridge" / "shoulder_calibration.json"


def current_emg_thresholds(path=CALIBRATION_FILE):
    """(grip, release) from the calibration run_demo_live.py saved, or None (then the summary shows levels only)."""
    try:
        saved = json.loads(Path(path).read_text())
        on = float(saved["emg_threshold"])
        return on, float(saved.get("emg_release_threshold", on))
    except (OSError, ValueError, KeyError):
        return None


def main():
    import serial
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from usb_serial_port import autodetect_port
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    parser.add_argument("--set", default=DEFAULT_SET, choices=sorted(PHASE_SETS),
                        help="filter = the 1-euro tuning set (default); base_raise = raising vs swinging, for the base; "
                             "emg = relaxed/gripping, still and while the arm moves, for the grip thresholds")
    args = parser.parse_args()
    phase_list = PHASE_SETS[args.set]
    port = args.port or autodetect_port()
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except serial.SerialException as exc:
        sys.exit(f"打不開序列埠 {port}：{exc}\n最常見的原因：run_demo_live.py 或 watch_imu_raw.py 還開著——先關掉再執行。")
    read_line = lambda: ser.readline().decode(errors="ignore")
    try:
        print(f"讀取 {port}（不燒錄、只讀）。共 {len(phase_list)} 段，每段都可以重錄。Ctrl+C 可隨時結束。")
        if not wait_for_data(read_line, time.monotonic):
            sys.exit(1)
        phases = run_session(read_line, time.monotonic, phases=phase_list)
    except KeyboardInterrupt:
        sys.exit("\n中斷，沒有存檔。")
    finally:
        ser.close()
    out = DATA_DIR / f"arm_motion_{datetime.now().strftime('%Y%m%d-%H%M%S')}.json"
    save(out, phases, port)
    print("\n".join(summary_lines(phases, emg_thresholds=current_emg_thresholds())))
    print(f"\n存到 {out}\n把這個檔名告訴 Claude，就能用它來決定參數。")


if __name__ == "__main__":
    main()

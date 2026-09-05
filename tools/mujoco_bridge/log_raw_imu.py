"""Interactive raw-IMU capture tool -- no MuJoCo, no mjpython, no zero-pose
calibration handshake. Walks through a fixed sequence of poses, prompting
"return to rest" (waits for Enter -- only asked for while both hands are
free, never mid-motion) then "now do X, recording automatically" (a fixed
countdown + hold, no keypress needed while the arm is actually posed).

Why raw values, not the firmware's decoded shoulder_pitch/shoulder_roll:
those go through ComplementaryFilter's gyro integration, which a live
session on 2026-09-03 showed drifting by several radians with ZERO
corresponding accelerometer change (raw values held rock-steady while
decoded roll drifted to -5.9rad) -- a real, currently-unverified gyro-
pairing bug (see phase3_control_loop_main.cpp's "NOT YET RE-VERIFIED"
comment). Raw accel readings aren't run through that integration at all,
so they're immune to it -- this tool sidesteps the whole problem rather
than working around it, and also means no viewer/MuJoCo/zero-pose wait is
needed to get usable ground-truth data for
tools/mujoco_bridge/test_imu_to_mujoco.py's fixtures.

Usage:
    python3 tools/mujoco_bridge/log_raw_imu.py
    python3 tools/mujoco_bridge/log_raw_imu.py --port /dev/tty.usbserial-0001 --baud 115200
"""

import argparse
import json
import re
import sys
import threading
import time
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PORT = "/dev/tty.usbserial-0001"
DEFAULT_BAUD = 115200

# Same fields as run_demo_live.py's SHOULDER_RAW_RE/ELBOW_RAW_RE -- kept as
# a separate copy (not imported) so this tool has zero dependency on
# mujoco/mjpython and can run under plain python3.
RAW_RE = re.compile(
    r"elbow_raw_ax=(?P<elbow_raw_ax>[-\d.eE+]+) elbow_raw_ay=(?P<elbow_raw_ay>[-\d.eE+]+) "
    r"elbow_raw_az=(?P<elbow_raw_az>[-\d.eE+]+).*?"
    r"shoulder_raw_ax=(?P<shoulder_raw_ax>[-\d.eE+]+) shoulder_raw_ay=(?P<shoulder_raw_ay>[-\d.eE+]+) "
    r"shoulder_raw_az=(?P<shoulder_raw_az>[-\d.eE+]+)"
)

# 2026-09-05: both raised (was 4.0/1.5) after real hardware data showed the
# OLD tail window wasn't long enough for a deliberately EXAGGERATED
# calibration pose (see run_demo_live.py's MIN_CALIBRATION_TILT_DEG comment
# for why exaggerated poses are wanted) -- comparing the first half of the
# old 1.5s tail against its second half showed real, still-ongoing drift
# (PURE_DOWN's az moved another -0.14 WITHIN the supposedly-settled window),
# meaning a big effortful reach can still be settling into position most of
# the way through a short recording, not just during an initial "moving"
# phase. average_tail() below now also warns if this is still happening.
RECORD_SECONDS = 6.0
SETTLE_TAIL_SECONDS = 2.5  # average only the last N seconds of the hold -- gives the motion time to finish and settle before it counts

# (pose name, spoken instruction) -- edit this list to capture a different
# set of poses. Order matters only for the printed prompts, not the output.
# 2026-09-05, second pass: testing whether adding a deliberate forearm/
# shoulder twist (thumb up when reaching left, thumb down when reaching
# right) to the LEFT/RIGHT reach gives the shoulder (bicep-mounted) sensor
# a distinguishable signal it otherwise wouldn't have -- a pure horizontal
# shoulder sweep rotates mostly about an axis close to parallel with
# gravity, which a single accelerometer fundamentally cannot see (the same
# "twist about its own gravity-sensing axis is unobservable" limit
# discussed for the sanity-check work). The first 4 poses are candidates
# for the ACTUAL future calibration sequence if this works; LEFT_NO_TWIST
# is a comparison-only control, not part of the intended calibration flow
# -- it exists purely to diff against LEFT_TWIST and see whether the twist
# actually changes shoulder_raw by a meaningful amount, or only elbow_raw
# (which would mean the twist is happening at the wrist, not the shoulder).
POSES = [
    ("BASELINE", "手臂自然垂下,手肘打直(這就是基準姿勢本身,零點)"),
    ("FORWARD", "從 BASELINE 手臂往前伸直,手肘打直,不轉手腕"),
    ("LEFT_TWIST", "從 BASELINE 手臂往左甩到底,手肘打直,同時大拇指轉朝上"),
    ("RIGHT_TWIST", "從 BASELINE 手臂往右甩到底,手肘打直,同時大拇指轉朝下"),
    ("LEFT_NO_TWIST", "(對照組,非校正姿勢) 從 BASELINE 手臂往左甩到底,手肘打直,手腕不要轉,跟 LEFT_TWIST 比較用"),
]


class LatestRaw:
    def __init__(self):
        self._lock = threading.Lock()
        self.shoulder = None  # (ax, ay, az)
        self.elbow = None

    def update(self, shoulder, elbow):
        with self._lock:
            self.shoulder = shoulder
            self.elbow = elbow

    def snapshot(self):
        with self._lock:
            return self.shoulder, self.elbow


def reader_thread_main(ser, latest):
    buf = b""
    while True:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\r\n" in buf:
            raw, buf = buf.split(b"\r\n", 1)
            line = raw.decode("utf-8", errors="ignore")
            m = RAW_RE.search(line)
            if not m:
                continue
            shoulder = (
                float(m.group("shoulder_raw_ax")),
                float(m.group("shoulder_raw_ay")),
                float(m.group("shoulder_raw_az")),
            )
            elbow = (
                float(m.group("elbow_raw_ax")),
                float(m.group("elbow_raw_ay")),
                float(m.group("elbow_raw_az")),
            )
            latest.update(shoulder, elbow)


def wait_for_first_sample(latest, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if latest.snapshot()[0] is not None:
            return True
        time.sleep(0.05)
    return False


def record_pose(latest, seconds):
    """Collects (timestamp, shoulder, elbow) samples for `seconds`, printing
    a countdown. Returns the full sample list."""
    samples = []
    t_start = time.time()
    t_end = t_start + seconds
    last_shown = None
    while time.time() < t_end:
        remaining = t_end - time.time()
        shown = int(remaining) + 1
        if shown != last_shown:
            print(f"  ...錄製中,還剩 {shown} 秒", flush=True)
            last_shown = shown
        shoulder, elbow = latest.snapshot()
        if shoulder is not None:
            samples.append((time.time() - t_start, shoulder, elbow))
        time.sleep(0.02)
    return samples


def average_tail(samples, tail_seconds):
    """Averages shoulder/elbow raw vectors over the samples whose timestamp
    falls in the last `tail_seconds` of the recording -- gives the motion
    time to finish moving and settle before any of it counts toward the
    reported pose, same reasoning as test_imu_to_mujoco.py's HOLD_TICKS
    convergence-window approach, just applied to real samples instead of a
    synthetic fixture.

    Also warns (2026-09-05) if the tail window itself still shows real
    drift -- splits the tail in half and compares the two halves' averages,
    since real hardware data showed the "settled" window can still be
    mid-settle for a big effortful pose (see this file's RECORD_SECONDS
    comment). This is a printed warning, not an automatic retry (unlike
    run_demo_live.py's MIN_CALIBRATION_TILT_DEG guard) -- read it and judge
    whether to redo the pose with more hold time."""
    if not samples:
        return None, None
    t_max = samples[-1][0]
    tail = [s for s in samples if s[0] >= t_max - tail_seconds]
    if not tail:
        tail = samples[-5:]
    n = len(tail)
    sx = sum(s[1][0] for s in tail) / n
    sy = sum(s[1][1] for s in tail) / n
    sz = sum(s[1][2] for s in tail) / n
    ex = sum(s[2][0] for s in tail) / n
    ey = sum(s[2][1] for s in tail) / n
    ez = sum(s[2][2] for s in tail) / n

    if len(tail) >= 4:
        t_mid = (tail[0][0] + tail[-1][0]) / 2.0
        first_half = [s[1] for s in tail if s[0] < t_mid]
        second_half = [s[1] for s in tail if s[0] >= t_mid]
        if first_half and second_half:
            fh = [sum(v[i] for v in first_half) / len(first_half) for i in range(3)]
            sh = [sum(v[i] for v in second_half) / len(second_half) for i in range(3)]
            drift = max(abs(sh[i] - fh[i]) for i in range(3))
            if drift > 0.05:
                print(f"  警告:這段錄製的『穩定期』內部,shoulder raw 還在漂移"
                      f"(前半段 vs 後半段最大差異 {drift:.3f}g)——這個姿勢可能還沒真的定住,"
                      f"考慮重錄、保持動作更久再結束。")

    return (sx, sy, sz), (ex, ey, ez)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--out", default=str(REPO_ROOT / "tools" / "mujoco_bridge" / "raw_imu_capture.json"))
    parser.add_argument("--repeats", type=int, default=1,
                         help="Capture each pose this many times in a row (return to rest between "
                              "each repeat) instead of once. With --repeats>1, results[name] is a "
                              "list of capture dicts instead of a single dict -- used to check "
                              "raw-sensor/human-repeatability noise, not the normal single-capture "
                              "fixture shape other tools (test_imu_to_mujoco.py) expect.")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port} @ {args.baud} baud")

    latest = LatestRaw()
    reader = threading.Thread(target=reader_thread_main, args=(ser, latest), daemon=True)
    reader.start()

    print("等待第一筆感測器資料...")
    if not wait_for_first_sample(latest):
        sys.exit("10 秒內沒收到任何資料,檢查硬體連線/電源後重試")
    print("資料流正常,開始。\n")

    # Resume support: load whatever's already saved (e.g. from a run that
    # got interrupted -- see the incremental save below, added 2026-09-03
    # after a run terminated mid-way through and lost everything, since the
    # old version only wrote the file once at the very end) and skip any
    # pose already present, so a rerun of the same command picks up where
    # it left off instead of re-asking for poses already captured.
    out_path = Path(args.out)
    results = {}
    if out_path.exists():
        with open(out_path) as f:
            results = json.load(f)
        if results:
            print(f"發現先前的錄製結果({', '.join(results.keys())}),會跳過這些、只錄剩下的。\n")

    for name, instruction in POSES:
        done = len(results.get(name, [])) if args.repeats > 1 else (1 if name in results else 0)
        if done >= args.repeats:
            print(f"=== {name} === (已有資料,跳過)")
            continue
        for rep in range(done, args.repeats):
            label = f"{name} ({rep + 1}/{args.repeats})" if args.repeats > 1 else name
            print(f"=== {label} ===")
            # 2026-09-05: this used to hardcode "手垂下" as the universal
            # return-to-neutral instruction before every pose, which made
            # sense when POSES[0] (REST) really was "arm hangs down" --
            # but broke silently once POSES[0] became BASELINE ("arm
            # straight forward" for the constrained grasp task): every
            # subsequent pose kept telling the person to hang the arm down
            # first, contradicting the actual intended baseline. Derived
            # from POSES[0] itself now instead of a separate hardcoded
            # string, so it can't drift out of sync with the pose list
            # again. POSES[0] itself (the reference pose) has no prior
            # pose to return to, so it skips this line entirely.
            if name == POSES[0][0]:
                print(f"請擺出「{instruction}」的姿勢。")
            else:
                print(f"請先回到「{POSES[0][0]}」姿勢({POSES[0][1]})。")
            print("準備好後按 Enter。")
            input()
            print(f"3 秒後開始 -- 接下來請做:「{instruction}」,並保持住直到錄製結束。")
            for n in (3, 2, 1):
                print(f"  {n}...", flush=True)
                time.sleep(1.0)
            print("開始錄製!請維持姿勢。")
            samples = record_pose(latest, RECORD_SECONDS)
            shoulder_avg, elbow_avg = average_tail(samples, SETTLE_TAIL_SECONDS)
            if shoulder_avg is None:
                print("  警告:這段完全沒收到資料,跳過。")
                continue
            print(f"  完成 -- shoulder raw(ax,ay,az)=({shoulder_avg[0]:+.3f},{shoulder_avg[1]:+.3f},{shoulder_avg[2]:+.3f})  "
                  f"elbow raw(ax,ay,az)=({elbow_avg[0]:+.3f},{elbow_avg[1]:+.3f},{elbow_avg[2]:+.3f})\n")
            entry = {
                "shoulder_raw_avg": shoulder_avg,
                "elbow_raw_avg": elbow_avg,
                "n_samples": len(samples),
                "all_samples": [
                    {"t": t, "shoulder": s, "elbow": e} for (t, s, e) in samples
                ],
            }
            if args.repeats > 1:
                results.setdefault(name, []).append(entry)
            else:
                results[name] = entry
            # Saved after EVERY repeat (not just once at the end, and not
            # just once per pose) -- see the resume-support comment above
            # main()'s loop for why: losing a whole session's data to one
            # interrupted run already happened once.
            with open(args.out, "w") as f:
                json.dump(results, f, indent=2)

    print(f"全部完成,結果存到 {args.out}")

    print("\n=== 總覽 ===")
    for name, data in results.items():
        entries = data if isinstance(data, list) else [data]
        for i, entry in enumerate(entries):
            s = entry["shoulder_raw_avg"]
            e = entry["elbow_raw_avg"]
            label = f"{name} ({i + 1}/{len(entries)})" if isinstance(data, list) else name
            print(f"{label:20s} shoulder=({s[0]:+.3f},{s[1]:+.3f},{s[2]:+.3f})  elbow=({e[0]:+.3f},{e[1]:+.3f},{e[2]:+.3f})")


if __name__ == "__main__":
    main()

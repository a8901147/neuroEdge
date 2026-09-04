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

RECORD_SECONDS = 4.0
SETTLE_TAIL_SECONDS = 1.5  # average only the last N seconds of the hold -- gives the motion time to finish and settle before it counts

# (pose name, spoken instruction) -- edit this list to capture a different
# set of poses. Order matters only for the printed prompts, not the output.
POSES = [
    ("REST", "維持垂下、手肘打直的姿勢不動(這就是基準姿勢本身)"),
    ("FORWARD_RAISE", "手肘打直,整支手臂往前舉到最高"),
    ("ABDUCTION_LEFT", "手肘打直,整支手臂往左側抬起"),
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
    synthetic fixture."""
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
    return (sx, sy, sz), (ex, ey, ez)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--out", default=str(REPO_ROOT / "tools" / "mujoco_bridge" / "raw_imu_capture.json"))
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
        if name in results:
            print(f"=== {name} === (已有資料,跳過)")
            continue
        print(f"=== {name} ===")
        print("請把手回到原位,手垂下、手肘打直。準備好後按 Enter。")
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
        results[name] = {
            "shoulder_raw_avg": shoulder_avg,
            "elbow_raw_avg": elbow_avg,
            "n_samples": len(samples),
            "all_samples": [
                {"t": t, "shoulder": s, "elbow": e} for (t, s, e) in samples
            ],
        }
        # Saved after EVERY pose (not just once at the end) -- see the
        # resume-support comment above main()'s loop for why: losing a
        # whole session's data to one interrupted run already happened once.
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2)

    print(f"全部完成,結果存到 {args.out}")

    print("\n=== 總覽 ===")
    for name, data in results.items():
        s = data["shoulder_raw_avg"]
        e = data["elbow_raw_avg"]
        print(f"{name:20s} shoulder=({s[0]:+.3f},{s[1]:+.3f},{s[2]:+.3f})  elbow=({e[0]:+.3f},{e[1]:+.3f},{e[2]:+.3f})")


if __name__ == "__main__":
    main()

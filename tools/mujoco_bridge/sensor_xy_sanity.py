"""Simplest possible sensor->MuJoCo XY-position check (2026-09-05).

Even simpler than sensor_orientation_sanity.py's rotation test: no
quaternion, no "which way is down" reasoning, no twist ambiguity to
worry about. Each shape's orientation is permanently fixed (its joints
are two `slide` axes, not a `ball` joint) -- it can only translate in
the XY plane. raw_ax feeds X, raw_ay feeds Y, raw_az is ignored
entirely. Move the physical sensor left/right/forward/back (without
rotating it) and watch whether the shape slides the same way on screen.

Usage:
    mjpython tools/mujoco_bridge/sensor_xy_sanity.py
    mjpython tools/mujoco_bridge/sensor_xy_sanity.py --port /dev/tty.usbserial-0001
"""

import argparse
import re
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import serial

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "sensor_xy_sanity.xml"

DEFAULT_PORT = "/dev/tty.usbserial-0001"
DEFAULT_BAUD = 115200

# raw_ax/ay are in g's; SCALE converts g -> meters of on-screen slide.
# RANGE must match the slide joints' range="-0.4 0.4" in the .xml.
SCALE = 0.3
RANGE = 0.4

RAW_RE = re.compile(
    r"elbow_raw_ax=(?P<elbow_raw_ax>[-\d.eE+]+) elbow_raw_ay=(?P<elbow_raw_ay>[-\d.eE+]+) "
    r"elbow_raw_az=(?P<elbow_raw_az>[-\d.eE+]+).*?"
    r"shoulder_raw_ax=(?P<shoulder_raw_ax>[-\d.eE+]+) shoulder_raw_ay=(?P<shoulder_raw_ay>[-\d.eE+]+) "
    r"shoulder_raw_az=(?P<shoulder_raw_az>[-\d.eE+]+)"
)

DIAG_LINE_RE = re.compile(
    r"diag shoulder_completions=(?P<shoulder_completions>\d+) "
    r"elbow_completions=(?P<elbow_completions>\d+) "
    r"shoulder_nacks=(?P<shoulder_nacks>\d+) shoulder_timeouts=(?P<shoulder_timeouts>\d+) "
    r"elbow_nacks=(?P<elbow_nacks>\d+) elbow_timeouts=(?P<elbow_timeouts>\d+)"
)


class Latest:
    def __init__(self):
        self._lock = threading.Lock()
        self.shoulder = None
        self.elbow = None
        self.diag = None

    def update(self, shoulder, elbow):
        with self._lock:
            self.shoulder = shoulder
            self.elbow = elbow

    def update_diag(self, diag):
        with self._lock:
            self.diag = diag

    def snapshot(self):
        with self._lock:
            return self.shoulder, self.elbow

    def snapshot_diag(self):
        with self._lock:
            return self.diag


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
            if m:
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
                continue
            diag_m = DIAG_LINE_RE.search(line)
            if diag_m:
                latest.update_diag((
                    int(diag_m.group("shoulder_completions")),
                    int(diag_m.group("elbow_completions")),
                    int(diag_m.group("shoulder_nacks")),
                    int(diag_m.group("shoulder_timeouts")),
                    int(diag_m.group("elbow_nacks")),
                    int(diag_m.group("elbow_timeouts")),
                ))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    if not SCENE_XML.exists():
        sys.exit(f"scene not found: {SCENE_XML}")

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port} @ {args.baud} baud -- Ctrl+C to stop")

    latest = Latest()
    reader = threading.Thread(target=reader_thread_main, args=(ser, latest), daemon=True)
    reader.start()

    print("等待第一筆感測器資料...")
    deadline = time.time() + 10.0
    while latest.snapshot()[0] is None:
        if time.time() > deadline:
            sys.exit("10 秒內沒收到任何資料,檢查硬體連線/電源後重試")
        time.sleep(0.05)
    print("資料流正常。紅色方塊=肩膀、藍色圓盤=手肘,兩個形狀完全不會旋轉,"
          "只會在水平面上平移:raw_ax 控制 X、raw_ay 控制 Y。開始。\n")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    shoulder_x_adr = model.jnt_qposadr[model.joint("shoulder_x").id]
    shoulder_y_adr = model.jnt_qposadr[model.joint("shoulder_y").id]
    elbow_x_adr = model.jnt_qposadr[model.joint("elbow_x").id]
    elbow_y_adr = model.jnt_qposadr[model.joint("elbow_y").id]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_count = 0
        last_print = 0.0
        while viewer.is_running():
            shoulder_raw, elbow_raw = latest.snapshot()
            data.qpos[shoulder_x_adr] = clamp(shoulder_raw[0] * SCALE, -RANGE, RANGE)
            data.qpos[shoulder_y_adr] = clamp(shoulder_raw[1] * SCALE, -RANGE, RANGE)
            data.qpos[elbow_x_adr] = clamp(elbow_raw[0] * SCALE, -RANGE, RANGE)
            data.qpos[elbow_y_adr] = clamp(elbow_raw[1] * SCALE, -RANGE, RANGE)
            mujoco.mj_forward(model, data)
            if step_count % 3 == 0:
                viewer.sync()
            now = time.monotonic()
            if now - last_print >= 0.5:
                diag = latest.snapshot_diag()
                diag_str = "n/a (no diag line yet)" if diag is None else (
                    f"shoulder: completions={diag[0]} nacks={diag[2]} timeouts={diag[3]} | "
                    f"elbow: completions={diag[1]} nacks={diag[4]} timeouts={diag[5]}"
                )
                print(f"shoulder_raw={tuple(round(x,3) for x in shoulder_raw)} "
                      f"xy=({data.qpos[shoulder_x_adr]:.3f},{data.qpos[shoulder_y_adr]:.3f})  |  "
                      f"elbow_raw={tuple(round(x,3) for x in elbow_raw)} "
                      f"xy=({data.qpos[elbow_x_adr]:.3f},{data.qpos[elbow_y_adr]:.3f})")
                print(f"  [DIAG] {diag_str}")
                last_print = now
            step_count += 1
            time.sleep(0.02)


if __name__ == "__main__":
    main()

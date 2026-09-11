"""Simplest possible sensor->MuJoCo orientation check (2026-09-05).

Deliberately bypasses EVERYTHING built on top of raw accelerometer data so
far in this project -- no calibration, no BASELINE/DOWN/LEFT_A poses, no
oblique-basis decompose, no arm kinematics. Just: read each IMU's raw
(ax,ay,az), normalize it, and rotate a simple shape so its own "down" axis
points the same way -- the shortest possible rotation from world-up to
that measured direction. Tilt the physical sensor, watch whether the
shape in MuJoCo tilts the same way.

This answers ONE question only: can a physical sensor's raw tilt be
faithfully reflected as a MuJoCo orientation at all? It does NOT attempt
to resolve the "twist about the sensor's own axis" ambiguity a single
accelerometer can't see (see SESSION_LOG.md's Session Handoff) -- the shape's
rotation about its own down-pointing axis is arbitrary (whatever the
shortest-rotation construction happens to pick), not meaningful. Only the
TILT (which way "down" points) is meaningful here.

Usage:
    mjpython tools/mujoco_bridge/sensor_orientation_sanity.py
    mjpython tools/mujoco_bridge/sensor_orientation_sanity.py --port /dev/tty.usbserial-0001
"""

import argparse
import math
import re
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import serial

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "sensor_orientation_sanity.xml"

DEFAULT_PORT = "/dev/tty.usbserial-0001"
DEFAULT_BAUD = 115200

RAW_RE = re.compile(
    r"elbow_raw_ax=(?P<elbow_raw_ax>[-\d.eE+]+) elbow_raw_ay=(?P<elbow_raw_ay>[-\d.eE+]+) "
    r"elbow_raw_az=(?P<elbow_raw_az>[-\d.eE+]+).*?"
    r"shoulder_raw_ax=(?P<shoulder_raw_ax>[-\d.eE+]+) shoulder_raw_ay=(?P<shoulder_raw_ay>[-\d.eE+]+) "
    r"shoulder_raw_az=(?P<shoulder_raw_az>[-\d.eE+]+)"
)

# Same pattern as run_demo_live.py's DIAG_LINE_RE -- added 2026-09-05 after
# a real session showed elbow_raw frozen at one stale value (never updating
# at all) while shoulder_raw kept tracking real motion fine. completions/
# nacks/timeouts are a real bus-level ACK/NACK signal straight from
# firmware, not a threshold on decoded values -- 0 completions with nonzero
# nacks means that reader's I2C address genuinely isn't answering (a wiring
# problem after remounting, not a software bug in this script).
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
        self.diag = None  # (shoulder_completions, elbow_completions, shoulder_nacks, shoulder_timeouts, elbow_nacks, elbow_timeouts)

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


def shortest_rotation_quat(ref, target):
    """Quaternion (w,x,y,z, MuJoCo's convention) for the shortest rotation
    taking unit vector `ref` to unit vector `target`. Standard
    half-angle-of-the-angle-between construction; falls back to a fixed
    perpendicular axis in the (measure-zero, never exactly hit with real
    sensor noise) case where target is exactly opposite ref."""
    rx, ry, rz = ref
    tx, ty, tz = target
    dot = rx * tx + ry * ty + rz * tz
    dot = max(-1.0, min(1.0, dot))
    if dot < -0.999999:
        # ref and target point exactly opposite -- any perpendicular axis
        # works for a 180deg rotation. Picks one deterministically.
        axis = (1.0, 0.0, 0.0) if abs(rx) < 0.9 else (0.0, 1.0, 0.0)
        cx, cy, cz = ry * axis[2] - rz * axis[1], rz * axis[0] - rx * axis[2], rx * axis[1] - ry * axis[0]
    else:
        cx, cy, cz = ry * tz - rz * ty, rz * tx - rx * tz, rx * ty - ry * tx
    clen = math.sqrt(cx * cx + cy * cy + cz * cz)
    angle = math.acos(dot)
    if clen < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)  # ref == target, no rotation
    cx, cy, cz = cx / clen, cy / clen, cz / clen
    half = angle / 2.0
    s = math.sin(half)
    return (math.cos(half), cx * s, cy * s, cz * s)


def normalize(v):
    mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / mag, v[1] / mag, v[2] / mag)


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
    print("資料流正常。紅色條紋=局部+X方向、藍色條紋=局部+Y方向,肩膀=方板、手肘=圓盤。開始。\n")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    shoulder_qpos_adr = model.jnt_qposadr[model.joint("shoulder_ball").id]
    elbow_qpos_adr = model.jnt_qposadr[model.joint("elbow_ball").id]

    # World "up" reference -- the shape's own down-pointing local axis
    # (implicitly its local -Z, since MuJoCo qpos identity quaternion
    # leaves the geom in its authored orientation and the plate/disc
    # geoms above are authored flat, normal along Z) is rotated to point
    # along whatever direction the sensor currently reads as gravity.
    WORLD_UP = (0.0, 0.0, 1.0)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_count = 0
        last_print = 0.0
        while viewer.is_running():
            shoulder_raw, elbow_raw = latest.snapshot()
            shoulder_q = shortest_rotation_quat(WORLD_UP, normalize(shoulder_raw))
            elbow_q = shortest_rotation_quat(WORLD_UP, normalize(elbow_raw))
            data.qpos[shoulder_qpos_adr:shoulder_qpos_adr + 4] = shoulder_q
            data.qpos[elbow_qpos_adr:elbow_qpos_adr + 4] = elbow_q
            mujoco.mj_forward(model, data)
            if step_count % 3 == 0:
                viewer.sync()
            # Diagnostic print (2026-09-05): prints the actual raw values
            # and the quaternion computed FROM them every ~0.5s, so a
            # "the shape isn't moving" report can be checked against
            # whether the underlying data itself is changing at all,
            # before suspecting the rendering/viewer side -- same
            # raw-data-first principle as everywhere else in this project.
            now = time.monotonic()
            if now - last_print >= 0.5:
                diag = latest.snapshot_diag()
                diag_str = "n/a (no diag line yet)" if diag is None else (
                    f"shoulder: completions={diag[0]} nacks={diag[2]} timeouts={diag[3]} | "
                    f"elbow: completions={diag[1]} nacks={diag[4]} timeouts={diag[5]}"
                )
                print(f"shoulder_raw={tuple(round(x,3) for x in shoulder_raw)} "
                      f"quat={tuple(round(x,3) for x in shoulder_q)}  |  "
                      f"elbow_raw={tuple(round(x,3) for x in elbow_raw)} "
                      f"quat={tuple(round(x,3) for x in elbow_q)}")
                print(f"  [DIAG] {diag_str}")
                last_print = now
            step_count += 1
            time.sleep(0.02)


if __name__ == "__main__":
    main()

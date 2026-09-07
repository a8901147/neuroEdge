"""First real-hardware test of MyoWare 2.0 -- deliberately independent of
both IMUs (neither is wired up yet, per the actual current bench setup:
only MyoWare is connected). run_demo_live.py can't be reused as-is for
this: its startup blocks on a real shoulder_raw sample and its calibration
flow normalizes that raw vector (divide-by-zero if the shoulder IMU is
truly never connected, since the firmware initializes shoulder_raw_ax/ay/az
to 0.0 and only overwrites them on a successful I2C read -- see
phase3_control_loop_main.cpp's own field declarations).

This script only reads grip=/gripping=/emg_min=/emg_max= from the live
UART stream (all on the same tick= line phase3_control_loop_main.cpp
already sends -- no firmware change needed) and holds the arm fixed at
grasp_test_common.py's validated front_left REACH_CTRL, with the same
grasp object placed there. No calibration, no shoulder/elbow tracking --
that's an orthogonal, already-tested concern (see PRD.md). This tests
exactly one thing end to end on real hardware: does flexing the real
muscle -> real MyoWare ENV signal -> GripStateMachine+SlewRateLimiter (on
the STM32, not a Python port) -> MuJoCo hand actually close around and
hold the object.

emg_min/emg_max (raw 12-bit ADC window min/max, before any threshold
comparison) are printed alongside the decoded grip/gripping specifically
so a real contraction can be confirmed in the RAW signal, not just
trusted from the decoded output -- same principle as this project's other
live diagnostic tools.

Usage:
    mjpython tools/mujoco_bridge/run_demo_live_grip_only.py
    mjpython tools/mujoco_bridge/run_demo_live_grip_only.py --port /dev/tty.usbserial-0001
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
import grasp_test_common as gtc  # noqa: E402
from run_demo_live import split_lines  # noqa: E402

DEFAULT_PORT = "/dev/tty.usbserial-0001"
DEFAULT_BAUD = 115200

# Deliberately only these four fields -- doesn't care about the rest of the
# tick= line (shoulder_pitch=/shoulder_roll=/elbow=/*_raw_* are all present
# too, since the firmware always sends the full line, but they're
# meaningless zeros here with no IMU connected and this script never reads
# them).
GRIP_LINE_RE = re.compile(
    r"grip=(?P<grip>[-\d.eE+]+) gripping=(?P<gripping>\d).*?"
    r"emg_min=(?P<emg_min>\d+) emg_max=(?P<emg_max>\d+)"
)


class LatestGrip:
    def __init__(self):
        self._lock = threading.Lock()
        self.grip = 0.0
        self.gripping = 0
        self.emg_min = None
        self.emg_max = None
        self.has_data = False

    def update(self, grip, gripping, emg_min, emg_max):
        with self._lock:
            self.grip = grip
            self.gripping = gripping
            self.emg_min = emg_min
            self.emg_max = emg_max
            self.has_data = True

    def snapshot(self):
        with self._lock:
            return self.grip, self.gripping, self.emg_min, self.emg_max, self.has_data


def reader_thread_main(ser, latest):
    buf = b""
    while True:
        chunk = ser.read(256)
        if not chunk:
            continue
        lines, buf = split_lines(buf, chunk)
        for line in lines:
            m = GRIP_LINE_RE.search(line)
            if m:
                latest.update(
                    float(m.group("grip")),
                    int(m.group("gripping")),
                    int(m.group("emg_min")),
                    int(m.group("emg_max")),
                )
                continue
            # 2026-09-07: this used to silently drop anything that didn't
            # match GRIP_LINE_RE -- meaning a real "no data" report gave
            # zero information about WHY (firmware never booted past its
            # banner? printing something in a different shape? nothing on
            # the wire at all?). Same reasoning as run_demo_live.py's own
            # [FW] passthrough: surface it instead of hiding it.
            if line.strip():
                print(f"[FW] {line}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port} @ {args.baud} baud -- Ctrl+C to stop")

    latest = LatestGrip()
    reader = threading.Thread(target=reader_thread_main, args=(ser, latest), daemon=True)
    reader.start()

    print("等待第一筆 grip 資料...")
    deadline = time.time() + 10.0
    while not latest.snapshot()[4]:
        if time.time() > deadline:
            sys.exit("10 秒內沒收到任何資料,檢查 MyoWare 接線/電源後重試")
        time.sleep(0.05)
    print("資料流正常。手臂固定在 front_left 姿勢,球已經放在指尖附近,開始。\n")

    model = gtc.load_model()
    data = mujoco.MjData(model)
    grip_ids = {name: model.actuator(name).id for name in gtc.GRIP_ACTUATORS}
    object_id = model.body("object").id

    for name, val in gtc.REACH_CTRL.items():
        data.ctrl[model.actuator(name).id] = val

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_count = 0
        last_print = 0.0
        while viewer.is_running():
            grip, gripping, emg_min, emg_max, _ = latest.snapshot()
            for name, upper_range in gtc.GRIP_ACTUATORS.items():
                data.ctrl[grip_ids[name]] = grip * gtc.GRIP_SCALE * upper_range
            mujoco.mj_step(model, data)
            if step_count % 20 == 0:
                viewer.sync()

            now = time.monotonic()
            if now - last_print >= 0.3:
                c = gtc.fingertip_centroid(model, data)
                o = data.xpos[object_id]
                dist = gtc._dist(c, o)
                print(f"emg_min={emg_min:4d} emg_max={emg_max:4d}  "
                      f"grip={grip:.3f} gripping={gripping}  "
                      f"dist_to_object={dist:.3f}m")
                last_print = now
            step_count += 1
            time.sleep(0.002)


if __name__ == "__main__":
    main()

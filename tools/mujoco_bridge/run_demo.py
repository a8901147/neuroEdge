"""Phase 2, iteration 1: drive a MuJoCo Shadow Hand simulation from the
real firmware control loop's logic (GripStateMachine + ComplementaryFilter +
SlewRateLimiter), replayed by src/mujoco_bridge_demo.cpp against a CSV fixture.

Must run as `mjpython run_demo.py`, not plain `python3` -- launch_passive
raises RuntimeError under plain CPython on macOS.
"""

import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parents[2]
CPP_BINARY = REPO_ROOT / "build" / "debug-heapguard" / "edgeneuro_mujoco_bridge_demo"
CSV_PATH = REPO_ROOT / "data" / "wearable_1emg_6imu.csv"
SCENE_XML = REPO_ROOT / "mujoco_menagerie" / "shadow_hand" / "scene_right.xml"

# Actuators driven directly by the smoothed grip scalar g in [0,1]: ctrl = g * upper_range.
# Abduction/thumb-base actuators are left at ctrl=0 (valid neutral) -- not driven this iteration.
GRIP_ACTUATORS = {
    "rh_A_FFJ3": 1.5708,
    "rh_A_MFJ3": 1.5708,
    "rh_A_RFJ3": 1.5708,
    "rh_A_LFJ3": 1.5708,
    "rh_A_FFJ0": 3.1415,
    "rh_A_MFJ0": 3.1415,
    "rh_A_RFJ0": 3.1415,
    "rh_A_LFJ0": 3.1415,
    "rh_A_THJ2": 0.6981,
    "rh_A_THJ1": 1.5708,
}

WRIST_ROLL_ACTUATOR = "rh_A_WRJ2"
WRIST_ROLL_RANGE = (-0.523599, 0.174533)

WRIST_PITCH_ACTUATOR = "rh_A_WRJ1"
WRIST_PITCH_RANGE = (-0.698132, 0.488692)

LINE_RE = re.compile(
    r"tick=(?P<tick>\d+) grip=(?P<grip>[-\d.eE+]+) gripping=(?P<gripping>\d) "
    r"roll=(?P<roll>[-\d.eE+]+) pitch=(?P<pitch>[-\d.eE+]+)"
)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class LatestSample:
    def __init__(self):
        self._lock = threading.Lock()
        self.grip = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.finished = False

    def update(self, grip, roll, pitch):
        with self._lock:
            self.grip = grip
            self.roll = roll
            self.pitch = pitch

    def mark_finished(self):
        with self._lock:
            self.finished = True

    def snapshot(self):
        with self._lock:
            return self.grip, self.roll, self.pitch, self.finished


def reader_thread_main(proc, latest):
    for line in proc.stdout:
        match = LINE_RE.search(line)
        if not match:
            continue
        latest.update(
            float(match.group("grip")),
            float(match.group("roll")),
            float(match.group("pitch")),
        )
    latest.mark_finished()


def main():
    if not CPP_BINARY.exists():
        sys.exit(f"bridge binary not found: {CPP_BINARY}\n"
                 f"build it first: cmake --build build/debug-heapguard "
                 f"--target edgeneuro_mujoco_bridge_demo")
    if not SCENE_XML.exists():
        sys.exit(f"shadow_hand scene not found: {SCENE_XML}\n"
                  f"fetch mujoco_menagerie/shadow_hand first (see README)")

    proc = subprocess.Popen(
        [str(CPP_BINARY), str(CSV_PATH)],
        stdout=subprocess.PIPE,
        text=True,
    )

    latest = LatestSample()
    reader = threading.Thread(target=reader_thread_main, args=(proc, latest), daemon=True)
    reader.start()

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    grip_actuator_ids = {
        name: model.actuator(name).id for name in GRIP_ACTUATORS
    }
    wrist_roll_id = model.actuator(WRIST_ROLL_ACTUATOR).id
    wrist_pitch_id = model.actuator(WRIST_PITCH_ACTUATOR).id

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            grip, roll, pitch, finished = latest.snapshot()

            for name, upper_range in GRIP_ACTUATORS.items():
                data.ctrl[grip_actuator_ids[name]] = grip * upper_range
            data.ctrl[wrist_roll_id] = clamp(roll, *WRIST_ROLL_RANGE)
            data.ctrl[wrist_pitch_id] = clamp(pitch, *WRIST_PITCH_RANGE)

            mujoco.mj_step(model, data)
            viewer.sync()

            if finished:
                # Hold last pose: keep syncing the viewer without stepping
                # physics further, so the window stays open and responsive.
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.1)
                break

            remaining = model.opt.timestep - (time.time() - step_start)
            if remaining > 0:
                time.sleep(remaining)

    proc.wait()


if __name__ == "__main__":
    main()

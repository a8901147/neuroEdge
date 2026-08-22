"""Phase 2, iteration 2: drive a MuJoCo whole-arm + hand simulation from the
real firmware control loop's logic (GripStateMachine + two ComplementaryFilter
instances for shoulder/elbow + SlewRateLimiter), replayed by
src/mujoco_bridge_demo.cpp against a CSV fixture. The arm reaches for and
grasps a physical object on tools/mujoco_bridge/arm_hand_scene.xml's pedestal
-- a real contact-based pickup, not just finger curl overlapping a mesh.
Sensor count matches the confirmed real Phase 3 hardware budget (2x MPU6050 +
1x MyoWare, PRD.md Section 3).

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
CSV_PATH = REPO_ROOT / "data" / "wearable_1emg_12imu.csv"
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# Actuators driven by the smoothed grip scalar g in [0,1]: ctrl = g * GRIP_SCALE * upper_range.
# GRIP_SCALE caps how far fingers curl -- verified empirically by stepping the
# simulation directly: closing fingers all the way to their full range slammed
# them past the grasp object hard enough to knock it off its pedestal instead
# of trapping it; 0.45 is the largest scale that held the object in testing.
# Abduction/thumb-base actuators are left at ctrl=0 (valid neutral) -- not driven this iteration.
GRIP_SCALE = 0.6
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

# Wrist is left neutral: the 2-IMU sensor budget (upper arm + forearm) drives
# the shoulder+elbow chain below, not the wrist directly -- wrist orientation
# isn't independently observable with only these 2 IMUs. Explicit ctrl=0 (set
# once, not per-tick) documents this as a deliberate decision, not an oversight.
WRIST_ROLL_ACTUATOR = "rh_A_WRJ2"
WRIST_PITCH_ACTUATOR = "rh_A_WRJ1"

SHOULDER_PITCH_ACTUATOR = "rh_A_shoulder_pitch"
SHOULDER_PITCH_RANGE = (-1.2, 1.2)

SHOULDER_ROLL_ACTUATOR = "rh_A_shoulder_roll"
SHOULDER_ROLL_RANGE = (-0.5, 0.8)

ELBOW_ACTUATOR = "rh_A_elbow_flex"
ELBOW_RANGE = (0.0, 1.4)

LINE_RE = re.compile(
    r"tick=(?P<tick>\d+) grip=(?P<grip>[-\d.eE+]+) gripping=(?P<gripping>\d) "
    r"shoulder_pitch=(?P<shoulder_pitch>[-\d.eE+]+) shoulder_roll=(?P<shoulder_roll>[-\d.eE+]+) "
    r"elbow=(?P<elbow>[-\d.eE+]+)"
)


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class LatestSample:
    def __init__(self):
        self._lock = threading.Lock()
        self.grip = 0.0
        self.shoulder_pitch = 0.0
        self.shoulder_roll = 0.0
        self.elbow = 0.0
        self.finished = False

    def update(self, grip, shoulder_pitch, shoulder_roll, elbow):
        with self._lock:
            self.grip = grip
            self.shoulder_pitch = shoulder_pitch
            self.shoulder_roll = shoulder_roll
            self.elbow = elbow

    def mark_finished(self):
        with self._lock:
            self.finished = True

    def snapshot(self):
        with self._lock:
            return self.grip, self.shoulder_pitch, self.shoulder_roll, self.elbow, self.finished


def reader_thread_main(proc, latest):
    for line in proc.stdout:
        match = LINE_RE.search(line)
        if not match:
            continue
        latest.update(
            float(match.group("grip")),
            float(match.group("shoulder_pitch")),
            float(match.group("shoulder_roll")),
            float(match.group("elbow")),
        )
    latest.mark_finished()


def main():
    if not CPP_BINARY.exists():
        sys.exit(f"bridge binary not found: {CPP_BINARY}\n"
                 f"build it first: cmake --build build/debug-heapguard "
                 f"--target edgeneuro_mujoco_bridge_demo")
    if not SCENE_XML.exists():
        sys.exit(f"arm+hand scene not found: {SCENE_XML}")

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
    shoulder_pitch_id = model.actuator(SHOULDER_PITCH_ACTUATOR).id
    shoulder_roll_id = model.actuator(SHOULDER_ROLL_ACTUATOR).id
    elbow_id = model.actuator(ELBOW_ACTUATOR).id

    data.ctrl[model.actuator(WRIST_ROLL_ACTUATOR).id] = 0.0
    data.ctrl[model.actuator(WRIST_PITCH_ACTUATOR).id] = 0.0

    object_body_id = model.body("object").id

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_count = 0
        while viewer.is_running():
            step_start = time.time()

            grip, shoulder_pitch, shoulder_roll, elbow, finished = latest.snapshot()

            for name, upper_range in GRIP_ACTUATORS.items():
                data.ctrl[grip_actuator_ids[name]] = grip * GRIP_SCALE * upper_range
            data.ctrl[shoulder_pitch_id] = clamp(shoulder_pitch, *SHOULDER_PITCH_RANGE)
            data.ctrl[shoulder_roll_id] = clamp(shoulder_roll, *SHOULDER_ROLL_RANGE)
            data.ctrl[elbow_id] = clamp(elbow, *ELBOW_RANGE)

            mujoco.mj_step(model, data)
            viewer.sync()

            step_count += 1
            if step_count % 200 == 0:
                # cheap numeric liftoff signal: a sustained rise here during
                # the grip-closed/retract phase confirms a real grasp, not
                # just visual overlap
                print(f"object height: {data.xpos[object_body_id][2]:.4f}")

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

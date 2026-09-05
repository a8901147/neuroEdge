"""Standalone grip/hand-closing kinematics check for arm_hand_scene.xml's
left hand -- no object, no serial port, no calibration. Sweeps the grip
scalar 0->1->0 (same GRIP_ACTUATORS mapping run_demo_live.py/run_demo.py
use) with the arm held in a fixed reach pose, printing each finger joint's
ctrl + resulting qpos at intervals. Confirms the hand can actually close and
open through its full mapped range -- no self-collision jam, no joint-limit
clipping surprise -- before ever testing it against a real object (that's
test_grasp_object.py; this one deliberately has no ball in it, so a problem
found here can't be confused with an object-interaction problem).

--headless runs without mujoco.viewer (plain python3, no display needed) --
the printed log is the same either way. Without --headless, must run as
`mjpython`, not plain `python3` -- launch_passive raises RuntimeError under
plain CPython on macOS (same constraint as run_demo_live.py/
test_arm_kinematics.py).

Usage:
    python3 tools/mujoco_bridge/test_grip_kinematics.py --headless
    mjpython tools/mujoco_bridge/test_grip_kinematics.py
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# Identical to run_demo_live.py's GRIP_SCALE/GRIP_ACTUATORS -- kept in sync
# by hand, same as that file's own copy of run_demo.py's constants. See
# run_demo_live.py's comment for the sign/range provenance (from
# unitree_g1/g1_with_hands.xml's own joint ranges, not guessed).
GRIP_SCALE = 0.6
GRIP_ACTUATORS = {
    "left_hand_thumb_1_joint": 1.0472,
    "left_hand_thumb_2_joint": 1.74533,
    "left_hand_middle_0_joint": -1.5708,
    "left_hand_middle_1_joint": -1.74533,
    "left_hand_index_0_joint": -1.5708,
    "left_hand_index_1_joint": -1.74533,
}

# A comfortable front-LEFT reach pose (nowhere near a joint-limit boundary)
# -- found via a real mj_step grid search, 2026-09-05, same search that also
# relocated arm_hand_scene.xml's pedestal/object to somewhere actually
# reachable (see that file's pedestal comment for the full story). This is
# the SAME pose test_grasp_object.py reaches to, so a hand-closing problem
# found here (no object involved) or there (object involved) can be told
# apart cleanly.
REACH_CTRL = {
    "left_shoulder_pitch_joint": -1.00,
    "left_shoulder_roll_joint": 0.80,
    "left_elbow_joint": 0.90,
}

SWEEP_PERIOD_SECONDS = 6.0
SYNC_EVERY_N_STEPS = 20  # ~25fps at timestep=0.002s, see test_arm_kinematics.py's own comment


def sweep_value(t, period):
    """Triangle wave 0->1->0 over `period` seconds."""
    phase = (t % period) / period
    return 2 * phase if phase < 0.5 else 2 * (1 - phase)


def run(model, data, viewer):
    """viewer=None runs headless -- see grasp_test_common.step_and_sync's
    comment for why (no display/mjpython needed, same printed log either
    way)."""
    for name, val in REACH_CTRL.items():
        data.ctrl[model.actuator(name).id] = val

    grip_actuator_ids = {name: model.actuator(name).id for name in GRIP_ACTUATORS}
    grip_qpos_adr = {
        name: model.jnt_qposadr[model.joint(name).id] for name in GRIP_ACTUATORS
    }

    print("Settling into reach pose (grip=0, hand open)...")
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    if viewer is not None:
        viewer.sync()

    print(f"\n[SWEEP] grip: 0 -> 1 -> 0 over {SWEEP_PERIOD_SECONDS:.0f}s (sim time) -- "
          f"watch the hand close into a fist and back open.")
    n_steps = int(SWEEP_PERIOD_SECONDS / model.opt.timestep)
    print_every = max(1, n_steps // 16)
    for i in range(n_steps):
        if viewer is not None and not viewer.is_running():
            break
        t = i * model.opt.timestep
        grip = sweep_value(t, SWEEP_PERIOD_SECONDS)
        for name, upper_range in GRIP_ACTUATORS.items():
            data.ctrl[grip_actuator_ids[name]] = grip * GRIP_SCALE * upper_range
        mujoco.mj_step(model, data)
        if viewer is not None and i % SYNC_EVERY_N_STEPS == 0:
            viewer.sync()
            time.sleep(model.opt.timestep * SYNC_EVERY_N_STEPS)
        if i % print_every == 0:
            joints_str = "  ".join(
                f"{name.replace('left_hand_', '').replace('_joint', '')}="
                f"{data.qpos[grip_qpos_adr[name]]:+.3f}"
                for name in GRIP_ACTUATORS
            )
            print(f"  grip={grip:.2f}  {joints_str}")

    print("\nSweep done -- no crash/jam through the full grip range.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    if not SCENE_XML.exists():
        raise SystemExit(f"scene not found: {SCENE_XML}")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    if args.headless:
        run(model, data, None)
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        run(model, data, viewer)
        print("Holding final pose, close the viewer window to exit.")
        i = 0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            if i % SYNC_EVERY_N_STEPS == 0:
                viewer.sync()
                time.sleep(model.opt.timestep * SYNC_EVERY_N_STEPS)
            i += 1


if __name__ == "__main__":
    main()

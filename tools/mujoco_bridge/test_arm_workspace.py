"""Reachable-workspace map for arm_hand_scene.xml's left arm -- no serial
port, no STM32, no viewer window (pure kinematics via mj_forward, so it runs
in well under a second with plain python3, not mjpython).

Answers "which directions can this arm point the hand in, using only
shoulder_pitch + shoulder_roll + elbow (our real 2-IMU sensor budget --
shoulder_yaw is fixed at 0, unobservable without a magnetometer)?" by
sweeping a grid of (pitch, roll) at two elbow bends and reporting the
resulting hand-pointing direction (azimuth around vertical, elevation from
horizontal) at every grid point, plus the same sweep WITH shoulder_yaw also
varied for comparison -- added 2026-09-02 after being asked for a test that
covers every direction a real human arm can reach, not just three single-
axis sweeps (see test_arm_kinematics.py, which checks direction/sign one
joint at a time but was never meant to answer a workspace-coverage
question).

Key finding this script exists to make concrete rather than asserted: a
"reach to the front-right" question isn't really a workspace-coverage
question at all -- the table below shows elbow-bent configurations reaching
almost the full 360deg of azimuth using only pitch+roll+elbow, no yaw
needed. The real limit is a SENSING one: reaching front-right with a
roughly-straight, roughly-horizontal arm is, for a real human shoulder,
predominantly a horizontal-flexion/yaw-like rotation -- and accelerometer-
only IMUs (gravity-referenced, no magnetometer) cannot observe rotation
about the gravity axis at all. The simulated arm CAN be posed pointing that
way (this script proves it can), but no natural reaching motion of a real
arm will drive it there through this sensor pipeline, because the real
motion that gets a human hand to that spot doesn't change the upper-arm
IMU's pitch/roll reading in the first place. Fixing that needs either a
9-axis IMU (magnetometer for yaw) or accepting the limitation -- not
something adjustable in this file or the firmware.

Usage:
    python3 tools/mujoco_bridge/test_arm_workspace.py
"""

from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# Real joint ranges (arm_hand_scene.xml) -- SHOULDER_PITCH_RANGE/
# SHOULDER_ROLL_RANGE match run_demo_live.py's clamp windows (2026-09-02:
# widened to real shoulder flexion/extension and ab/adduction ROM, capped
# by each joint's own mechanical limit -- see that file's comments), not
# the full mechanical joint limits on the OTHER side, since those are what
# real sensor data is actually clamped into. SHOULDER_PITCH_RANGE flipped
# 2026-09-03 along with run_demo_live.py's own fix -- negative ctrl is
# flexion/forward, positive is extension/backward, opposite of the first
# (unverified) guess.
SHOULDER_PITCH_RANGE = (-3.0892, 1.0472)
SHOULDER_ROLL_RANGE = (-0.8727, 2.2515)
SHOULDER_YAW_SWEEP = (-1.5, 1.5)  # hypothetical -- see module docstring
ELBOW_STRAIGHT = 1.28  # hanging pose, see run_demo_live.py's ELBOW_OFFSET
ELBOW_BENT = -0.5  # partial flexion toward the joint's other limit (-1.0472)
GRID_N = 9


def main():
    if not SCENE_XML.exists():
        raise SystemExit(f"scene not found: {SCENE_XML}")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    sp_adr = model.jnt_qposadr[model.joint("left_shoulder_pitch_joint").id]
    sr_adr = model.jnt_qposadr[model.joint("left_shoulder_roll_joint").id]
    sy_adr = model.jnt_qposadr[model.joint("left_shoulder_yaw_joint").id]
    el_adr = model.jnt_qposadr[model.joint("left_elbow_joint").id]
    shoulder_body = model.body("left_shoulder_roll_link").id
    wrist_body = model.body("left_wrist_yaw_link").id

    def direction_deg(pitch, roll, yaw, elbow):
        """(azimuth, elevation) of the shoulder->wrist vector, in degrees.
        azimuth: 0=straight ahead (+X), +90=fully across-body to the LEFT
        (+Y, the wearer's own left, since this is their left arm), -90=out
        to the RIGHT (-Y). elevation: +90=straight up, -90=straight down,
        0=horizontal."""
        mujoco.mj_resetData(model, data)
        data.qpos[sp_adr] = pitch
        data.qpos[sr_adr] = roll
        data.qpos[sy_adr] = yaw
        data.qpos[el_adr] = elbow
        mujoco.mj_forward(model, data)
        rel = data.xpos[wrist_body] - data.xpos[shoulder_body]
        rel_n = rel / np.linalg.norm(rel)
        azimuth = np.degrees(np.arctan2(rel_n[1], rel_n[0]))
        elevation = np.degrees(np.arcsin(np.clip(rel_n[2], -1, 1)))
        return azimuth, elevation

    pitches = np.linspace(*SHOULDER_PITCH_RANGE, GRID_N)
    rolls = np.linspace(*SHOULDER_ROLL_RANGE, GRID_N)

    for label, elbow in [("STRAIGHT", ELBOW_STRAIGHT), ("BENT", ELBOW_BENT)]:
        print(f"\n=== elbow={label} ({elbow:.3f} rad), yaw LOCKED at 0 "
              f"(our real 2-IMU limit) ===")
        print("     each cell: azimuth,elevation (deg) -- columns are roll "
              f"{rolls[0]:+.2f}..{rolls[-1]:+.2f}, rows are pitch")
        az_min, az_max = 1e9, -1e9
        for p in pitches:
            row = []
            for r in rolls:
                az, el = direction_deg(p, r, 0.0, elbow)
                az_min, az_max = min(az_min, az), max(az_max, az)
                row.append(f"{az:+4.0f},{el:+3.0f}")
            print(f"  pitch={p:+.2f}: " + "  ".join(row))
        print(f"  azimuth range covered (yaw locked): {az_min:.0f} to {az_max:.0f} deg")

    print("\n=== Same, but ALSO sweeping shoulder_yaw (what a magnetometer "
          "would unlock) ===")
    yaws = np.linspace(*SHOULDER_YAW_SWEEP, GRID_N)
    az_min, az_max = 1e9, -1e9
    for y in yaws:
        for p in (SHOULDER_PITCH_RANGE[0], 0.0, SHOULDER_PITCH_RANGE[1]):
            for r in (SHOULDER_ROLL_RANGE[0], 0.0, SHOULDER_ROLL_RANGE[1]):
                az, el = direction_deg(p, r, y, ELBOW_STRAIGHT)
                az_min, az_max = min(az_min, az), max(az_max, az)
    print(f"  azimuth range covered (yaw swept too): {az_min:.0f} to {az_max:.0f} deg")


if __name__ == "__main__":
    main()

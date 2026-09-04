"""Tests the tilt/azimuth -> Data->ctrl -> MuJoCo PIPELINE end-to-end, using
a SYNTHETIC (placeholder) calibration basis rather than a real one -- this
is deliberately decoupled from "is the real mount's forward direction
correctly calibrated" (that needs a real hardware capture: REST + a
forward-raise + an abduction-left in one sitting, so the basis can be
derived from real motion instead of guessed -- see PRD.md's Session
Handoff section). What THIS file proves is: given ANY valid orthonormal
calibration basis, does the rest of the pipeline (tilt/azimuth decode ->
pitch/roll equivalent -> ctrl clamp -> MuJoCo forward kinematics) produce
the anatomically correct direction. That part has no real-hardware
dependency at all, so there's no reason to leave it untested just because
the calibration capture hasn't happened yet.

Mirrors tests/test_tilt_azimuth.cpp's own reference vector and synthetic
poses (hang-down/forward/front-left/front-right) for cross-checking, and
adds abduction-left/adduction-right/backward-extension since those are
also part of this project's real coverage goal.

Once a real calibration session lands (log_raw_imu.py: REST, FORWARD_RAISE,
ABDUCTION_LEFT in one sitting, see PRD.md TODO #1), replace REF and BASIS_U
below with the measured values and this file's SYNTHETIC test cases can be
either kept (they test the pipeline logic, not the real mount) or
supplemented with a real-data version alongside test_imu_to_mujoco.py.

Usage:
    python3 tools/mujoco_bridge/test_tilt_azimuth_pipeline.py
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_demo_live import clamp, SHOULDER_PITCH_RANGE, SHOULDER_ROLL_RANGE  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# PLACEHOLDER calibration -- same non-axis-aligned reference vector
# test_tilt_azimuth.cpp uses, chosen there specifically to prove the math
# isn't only correct for a convenient axis-aligned case. NOT this mount's
# real REST reading (that's tools/mujoco_bridge/raw_imu_capture.json's
# "REST" entry) and basis_u is NOT a real measured "forward" direction --
# see module docstring. Replace both once the real calibration capture
# lands; nothing else in this file should need to change.
REF = np.array([0.6, -0.3, 0.74])
REF = REF / np.linalg.norm(REF)


def orthonormal_basis_perpendicular_to(ref):
    """Direct Python port of tilt_azimuth.hpp's function of the same name
    -- keep these two in sync if either changes."""
    seed = np.array([0.0, 0.0, 1.0])
    if abs(ref[2]) > 0.99:
        seed = np.array([1.0, 0.0, 0.0])
    u = np.cross(ref, seed)
    u = u / np.linalg.norm(u)
    v = np.cross(ref, u)
    return u, v


BASIS_U, BASIS_V = orthonormal_basis_perpendicular_to(REF)


def tilt_azimuth(accel, ref, u, v):
    """Direct Python port of tilt_azimuth.hpp's tilt_azimuth() -- keep in
    sync if either changes."""
    accel = accel / np.linalg.norm(accel)
    dot_ref = np.clip(np.dot(accel, ref), -1.0, 1.0)
    tilt = np.arccos(dot_ref)
    azimuth = np.arctan2(np.dot(accel, v), np.dot(accel, u))
    return tilt, azimuth


def synthesize(ref, u, v, tilt_deg, azimuth_deg):
    """Inverse of tilt_azimuth -- builds the accel vector a real sensor
    would read at the given (tilt, azimuth) from `ref`. Same construction
    as test_tilt_azimuth.cpp's synthesize()."""
    tilt = np.radians(tilt_deg)
    az = np.radians(azimuth_deg)
    return ref * np.cos(tilt) + (u * np.cos(az) + v * np.sin(az)) * np.sin(tilt)


def tilt_azimuth_to_ctrl(tilt, azimuth):
    """pitch/roll-equivalent from (tilt, azimuth), clamped into this
    project's real joint ranges -- see run_demo_live.py's
    SHOULDER_PITCH_RANGE/SHOULDER_ROLL_RANGE comments for where those
    numbers come from. Sign convention matches the already-verified
    Data->MuJoCo mapping: negative ctrl = forward, positive ctrl = left
    (see run_demo_live.py's own pitch/roll ctrl assignment comments)."""
    pitch_eq = tilt * np.cos(azimuth)
    roll_eq = tilt * np.sin(azimuth)
    pitch_ctrl = clamp(-pitch_eq, *SHOULDER_PITCH_RANGE)
    roll_ctrl = clamp(roll_eq, *SHOULDER_ROLL_RANGE)
    return pitch_ctrl, roll_ctrl


def mujoco_wrist_position(pitch_ctrl, roll_ctrl, elbow_ctrl=1.28):
    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    sp = model.jnt_qposadr[model.joint("left_shoulder_pitch_joint").id]
    sr = model.jnt_qposadr[model.joint("left_shoulder_roll_joint").id]
    el = model.jnt_qposadr[model.joint("left_elbow_joint").id]
    shoulder_body = model.body("left_shoulder_roll_link").id
    wrist_body = model.body("left_wrist_yaw_link").id
    data.qpos[sp] = pitch_ctrl
    data.qpos[sr] = roll_ctrl
    data.qpos[el] = elbow_ctrl
    mujoco.mj_forward(model, data)
    return data.xpos[wrist_body] - data.xpos[shoulder_body]  # (front, left, up)


def main():
    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)

    # (label, tilt_deg, azimuth_deg, expected front sign, expected left sign)
    # None means "don't check that component's sign" (e.g. hang-down has no
    # meaningful direction at tilt=0).
    cases = [
        ("HANG_DOWN", 0, 0, None, None),
        ("FORWARD_RAISE", 110, 0, "+", "~"),
        ("BACKWARD_EXTENSION", 60, 180, "-", "~"),
        ("ABDUCTION_LEFT", 110, 90, "~", "+"),
        ("ADDUCTION_RIGHT", 40, -90, "~", "-"),
        ("FRONT_LEFT_RAISE", 100, 45, "+", "+"),
        ("FRONT_RIGHT_RAISE", 100, -45, "+", "-"),
    ]

    print(f"{'pose':20s} {'tilt':>6s} {'azimuth':>8s}  {'pitch_ctrl':>10s} {'roll_ctrl':>9s}  mujoco(front,left,up)")
    for name, tilt_deg, az_deg, exp_front, exp_left in cases:
        accel = synthesize(REF, BASIS_U, BASIS_V, tilt_deg, az_deg)
        tilt, azimuth = tilt_azimuth(accel, REF, BASIS_U, BASIS_V)
        # Round-trip sanity: decoded (tilt,azimuth) must match what was synthesized.
        check(abs(np.degrees(tilt) - tilt_deg) < 0.1,
              f"{name}: round-trip tilt mismatch ({np.degrees(tilt):.2f} vs {tilt_deg})")

        pitch_ctrl, roll_ctrl = tilt_azimuth_to_ctrl(tilt, azimuth)
        rel = mujoco_wrist_position(pitch_ctrl, roll_ctrl)
        print(f"{name:20s} {tilt_deg:5d}deg {az_deg:+7d}deg  {pitch_ctrl:+10.4f} {roll_ctrl:+9.4f}  "
              f"({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})")

        if exp_front == "+":
            check(rel[0] > 0.05, f"{name}: expected clearly-forward wrist, got front={rel[0]:+.3f}")
        elif exp_front == "-":
            check(rel[0] < -0.05, f"{name}: expected clearly-behind wrist, got front={rel[0]:+.3f}")
        if exp_left == "+":
            check(rel[1] > 0.05, f"{name}: expected clearly-left wrist, got left={rel[1]:+.3f}")
        elif exp_left == "-":
            check(rel[1] < -0.05, f"{name}: expected clearly-right wrist, got left={rel[1]:+.3f}")

    if failures:
        print("\nFAILED:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("\nPASSED: tilt/azimuth -> ctrl -> MuJoCo pipeline gives the anatomically "
          "correct direction for all 7 synthetic poses (placeholder calibration basis).")


if __name__ == "__main__":
    main()

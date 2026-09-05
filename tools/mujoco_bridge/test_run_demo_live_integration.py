"""Integration test for run_demo_live.py's actual live pipeline: raw
shoulder/elbow readings -> BASELINE/FORWARD/LEFT_TWIST oblique-basis decode
-> raw-domain EMA smoothing -> rate-limited ctrl -> real MuJoCo forward
kinematics. Mirrors test_imu_to_mujoco.py's role (that file integration-
tests src/mujoco_bridge_demo.cpp's C++ CSV-replay path and its own,
deliberately-untouched REST/FORWARD_RAISE/ABDUCTION_LEFT calibration) but
for the Python live-hardware path and this session's BASELINE/FORWARD/
LEFT_TWIST calibration instead -- that combination (today's calibration
pivot + the rate limiter/EMA smoothing added the same session) had zero
coverage above the level of individual functions before this file:
test_run_demo_live_math.py checks each function in isolation, this checks
them wired together the same way main()'s loop actually calls them.

Uses SYNTHETIC but PHYSICALLY-PLAUSIBLE raw vectors (same reasoning as
test_imu_to_mujoco.py's own docstring: real captured calibration data
lives in shoulder_calibration.json, which is gitignored -- a committed
test can't depend on it being present). The synthetic BASELINE/FORWARD/
LEFT_TWIST directions are deliberately NOT 90deg apart (~55deg here),
mirroring this session's real finding that these directions aren't
orthogonal on a real body.

Usage:
    python3 tools/mujoco_bridge/test_run_demo_live_integration.py
"""

import sys
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_demo_live as rdl  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# Synthetic calibration raw vectors -- BASELINE is arm-hangs-down (gravity
# mostly along local -Z of the sensor's own frame here, but the pipeline
# never assumes any particular axis meaning, only real captured
# directions), FORWARD is a large tilt away from it, LEFT_TWIST is a
# different large tilt only ~55deg from FORWARD (not 90deg) -- same
# non-orthogonal-on-purpose shape as the real captured twist-check data
# (real LEFT_TWIST/RIGHT_TWIST turned out clearly separable via ay's sign,
# which this fixture also preserves).
BASELINE_RAW = (0.02, 0.05, 0.98)
FORWARD_RAW = (0.75, 0.10, 0.60)
LEFT_TWIST_RAW = (0.55, 0.65, 0.35)
RIGHT_TWIST_RAW = (0.55, -0.65, 0.35)

BASELINE_ELBOW = 0.30   # straight-ish arm, arbitrary "zero" reading
BENT_ELBOW = 1.45       # a real flexion away from baseline

# Ticks to hold each simulated pose -- long enough for both the EMA
# (RAW_SMOOTHING_ALPHA) and the rate limiter (MAX_CTRL_RATE_RAD_PER_SEC)
# to fully converge on a steady target, not just approach it. 1500 ticks
# at the scene's 2ms timestep is 3s of simulated hold time.
HOLD_TICKS = 1500


class LiveLoopState:
    """Re-implements exactly the per-tick sequence main()'s while loop
    runs (raw EMA -> oblique decode -> ctrl mapping -> rate limit),
    calling the SAME functions run_demo_live.py's real loop calls, in the
    same order -- not a reimplementation of their logic, just the wiring
    around them, so this test exercises what actually runs live."""

    def __init__(self, shoulder_basis, zero_elbow):
        self.shoulder_basis = shoulder_basis
        self.zero_elbow = zero_elbow
        self.smoothed_shoulder_raw = None
        self.smoothed_elbow_raw_scalar = None
        self.smoothed_pitch_ctrl = None
        self.smoothed_roll_ctrl = None
        self.smoothed_elbow_ctrl = None

    def tick(self, shoulder_raw, elbow, max_ctrl_step):
        if self.smoothed_shoulder_raw is None:
            self.smoothed_shoulder_raw = shoulder_raw
            self.smoothed_elbow_raw_scalar = elbow
        else:
            self.smoothed_shoulder_raw = tuple(
                rdl.ema_step(self.smoothed_shoulder_raw[i], shoulder_raw[i], rdl.RAW_SMOOTHING_ALPHA)
                for i in range(3)
            )
            self.smoothed_elbow_raw_scalar = rdl.ema_step(
                self.smoothed_elbow_raw_scalar, elbow, rdl.RAW_SMOOTHING_ALPHA)

        pitch_equiv, roll_equiv = rdl.oblique_decompose_scaled(self.shoulder_basis, self.smoothed_shoulder_raw)
        target_pitch_ctrl = rdl.clamp(-pitch_equiv, *rdl.SHOULDER_PITCH_RANGE)
        target_roll_ctrl = rdl.clamp(roll_equiv, *rdl.SHOULDER_ROLL_RANGE)
        target_elbow_ctrl = rdl.clamp(
            rdl.ELBOW_OFFSET - (self.smoothed_elbow_raw_scalar - self.zero_elbow), *rdl.ELBOW_RANGE)

        if self.smoothed_pitch_ctrl is None:
            self.smoothed_pitch_ctrl = target_pitch_ctrl
            self.smoothed_roll_ctrl = target_roll_ctrl
            self.smoothed_elbow_ctrl = target_elbow_ctrl
        else:
            self.smoothed_pitch_ctrl = rdl.rate_limit_step(self.smoothed_pitch_ctrl, target_pitch_ctrl, max_ctrl_step)
            self.smoothed_roll_ctrl = rdl.rate_limit_step(self.smoothed_roll_ctrl, target_roll_ctrl, max_ctrl_step)
            self.smoothed_elbow_ctrl = rdl.rate_limit_step(self.smoothed_elbow_ctrl, target_elbow_ctrl, max_ctrl_step)
        return self.smoothed_pitch_ctrl, self.smoothed_roll_ctrl, self.smoothed_elbow_ctrl


def hold_pose(model, data, state, shoulder_raw, elbow, pitch_id, roll_id, elbow_id, n_ticks=HOLD_TICKS):
    max_ctrl_step = rdl.MAX_CTRL_RATE_RAD_PER_SEC * model.opt.timestep
    for _ in range(n_ticks):
        pitch_ctrl, roll_ctrl, elbow_ctrl = state.tick(shoulder_raw, elbow, max_ctrl_step)
        data.ctrl[pitch_id] = pitch_ctrl
        data.ctrl[roll_id] = roll_ctrl
        data.ctrl[elbow_id] = elbow_ctrl
        mujoco.mj_step(model, data)


def main():
    failures = []

    def check(cond, msg):
        if not cond:
            failures.append(msg)

    # calibration_tilt_deg() already returns degrees -- see its own definition.
    fwd_tilt = rdl.calibration_tilt_deg(BASELINE_RAW, FORWARD_RAW)
    left_tilt = rdl.calibration_tilt_deg(BASELINE_RAW, LEFT_TWIST_RAW)
    fwd_left_separation = rdl.calibration_tilt_deg(FORWARD_RAW, LEFT_TWIST_RAW)
    print(f"Synthetic calibration: BASELINE-FORWARD tilt={fwd_tilt:.1f}deg, "
          f"BASELINE-LEFT_TWIST tilt={left_tilt:.1f}deg, "
          f"FORWARD-LEFT_TWIST separation={fwd_left_separation:.1f}deg (deliberately non-orthogonal)")
    check(fwd_left_separation < 80.0,
          f"fixture no longer non-orthogonal (separation={fwd_left_separation:.1f}deg) -- "
          f"this test is specifically meant to exercise the non-orthogonal case")

    shoulder_basis = rdl.make_oblique_basis(BASELINE_RAW, FORWARD_RAW, LEFT_TWIST_RAW)

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    pitch_id = model.actuator("left_shoulder_pitch_joint").id
    roll_id = model.actuator("left_shoulder_roll_joint").id
    elbow_id = model.actuator("left_elbow_joint").id
    shoulder_body = model.body("left_shoulder_roll_link").id
    wrist_body = model.body("left_wrist_yaw_link").id

    def wrist_rel():
        return data.xpos[wrist_body] - data.xpos[shoulder_body]

    # --- BASELINE: should settle near the model's own hang-down rest pose ---
    state = LiveLoopState(shoulder_basis, zero_elbow=BASELINE_ELBOW)
    hold_pose(model, data, state, BASELINE_RAW, BASELINE_ELBOW, pitch_id, roll_id, elbow_id)
    rel = wrist_rel()
    print(f"BASELINE          ctrl=({data.ctrl[pitch_id]:+.3f},{data.ctrl[roll_id]:+.3f})  "
          f"wrist(front,left,up)=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})")
    check(abs(data.ctrl[pitch_id]) < 0.05, f"BASELINE: expected ~0 pitch ctrl, got {data.ctrl[pitch_id]:+.3f}")
    check(abs(data.ctrl[roll_id]) < 0.05, f"BASELINE: expected ~0 roll ctrl, got {data.ctrl[roll_id]:+.3f}")

    # --- FORWARD: real shoulder flexion -> wrist clearly in front ---
    state = LiveLoopState(shoulder_basis, zero_elbow=BASELINE_ELBOW)
    hold_pose(model, data, state, FORWARD_RAW, BASELINE_ELBOW, pitch_id, roll_id, elbow_id)
    rel = wrist_rel()
    print(f"FORWARD           ctrl=({data.ctrl[pitch_id]:+.3f},{data.ctrl[roll_id]:+.3f})  "
          f"wrist(front,left,up)=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})")
    check(rel[0] > 0.1, f"FORWARD: expected clearly-forward wrist, got front={rel[0]:+.3f}")

    # --- LEFT_TWIST-like reach: wrist clearly to the left ---
    state = LiveLoopState(shoulder_basis, zero_elbow=BASELINE_ELBOW)
    hold_pose(model, data, state, LEFT_TWIST_RAW, BASELINE_ELBOW, pitch_id, roll_id, elbow_id)
    rel = wrist_rel()
    print(f"LEFT_TWIST        ctrl=({data.ctrl[pitch_id]:+.3f},{data.ctrl[roll_id]:+.3f})  "
          f"wrist(front,left,up)=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})")
    check(rel[1] > 0.1, f"LEFT_TWIST: expected clearly-left wrist, got left={rel[1]:+.3f}")

    # --- RIGHT_TWIST (held out of the basis, validation-only, same as
    # run_demo_live.py's own calibration-time check) -- must decode to the
    # opposite side from LEFT_TWIST. ---
    state = LiveLoopState(shoulder_basis, zero_elbow=BASELINE_ELBOW)
    hold_pose(model, data, state, RIGHT_TWIST_RAW, BASELINE_ELBOW, pitch_id, roll_id, elbow_id)
    rel = wrist_rel()
    print(f"RIGHT_TWIST       ctrl=({data.ctrl[pitch_id]:+.3f},{data.ctrl[roll_id]:+.3f})  "
          f"wrist(front,left,up)=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})")
    check(rel[1] < -0.1, f"RIGHT_TWIST: expected clearly-right wrist, got left={rel[1]:+.3f}")

    # --- Elbow flexion, independent of shoulder pose ---
    state = LiveLoopState(shoulder_basis, zero_elbow=BASELINE_ELBOW)
    hold_pose(model, data, state, BASELINE_RAW, BENT_ELBOW, pitch_id, roll_id, elbow_id)
    print(f"ELBOW_FLEXION     elbow_ctrl={data.ctrl[elbow_id]:+.3f}")
    check(data.ctrl[elbow_id] < rdl.ELBOW_OFFSET - 0.1,
          f"ELBOW_FLEXION: expected ctrl clearly below straight-arm ELBOW_OFFSET={rdl.ELBOW_OFFSET}, "
          f"got {data.ctrl[elbow_id]:+.3f}")

    if failures:
        print("\nFAILED:")
        for f in failures:
            print(f"  - {f}")
        sys.exit(1)
    print("\nPASSED: run_demo_live.py's real per-tick pipeline (raw EMA -> oblique decode -> "
          "rate-limited ctrl) gives the anatomically correct direction for all poses, "
          "using a deliberately non-orthogonal synthetic calibration basis.")


if __name__ == "__main__":
    main()

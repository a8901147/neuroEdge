"""Standalone kinematics check for arm_hand_scene.xml's left arm -- no serial
port, no STM32, no zero-pose calibration. Sweeps shoulder_pitch/roll and
elbow through their real joint ranges one at a time. Prints both the ctrl
target AND a numeric geometry_summary() (hang angle from vertical, elbow
flexion in degrees, wrist position relative to the shoulder) at regular
points through each sweep, so a "does it hang right / does the elbow bend
the right way"
question can be answered by reading this printed log alone -- added
2026-09-02 specifically so this doesn't require watching the interactive
viewer window in real time (which whoever runs this script can still do,
but the log alone should be enough to judge correctness without it).

Must run as `mjpython`, not plain `python3` -- launch_passive raises
RuntimeError under plain CPython on macOS (same constraint as
run_demo_live.py/run_demo.py).

Usage:
    mjpython tools/mujoco_bridge/test_arm_kinematics.py
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = REPO_ROOT / "tools" / "mujoco_bridge" / "arm_hand_scene.xml"

# (actuator name, real joint range, seconds per full sweep) -- one phase at a
# time, others held at rest, so a direction/hang question about one joint
# isn't muddled by the others also moving. Ranges match run_demo_live.py's
# SHOULDER_PITCH_RANGE/SHOULDER_ROLL_RANGE (2026-09-02: widened from an
# arbitrary carried-over window to real shoulder flexion/extension and
# ab/adduction ROM, capped by each joint's own mechanical limit -- see that
# file's comments for the anatomical reference values).
PHASES = [
    ("left_elbow_joint", (-1.0472, 1.28), 16.0),
    ("left_shoulder_pitch_joint", (-3.0892, 1.0472), 16.0),
    ("left_shoulder_roll_joint", (-0.8727, 2.2515), 16.0),
]
PHASE_HOLD_SECONDS = 2.0  # pause at rest between phases, arm fully still

# ctrl=0 on the elbow actuator is NOT "arm straight". Nor, it turns out, is
# -1.0472 (this file's own first guess, and run_demo_live.py's -- both
# wrong): that value was picked from the raw angle between the upper-arm
# and forearm body vectors alone, which ignores the extra shoulder_yaw_link
# offset sitting between them and doesn't actually track whether the WHOLE
# arm hangs down. +1.28 -- confirmed by direct visual comparison against
# the frozen right arm, which uses this exact value (G1's own "stand"
# keyframe) and clearly hangs straight -- is the real hang-down reference.
# See run_demo_live.py's ELBOW_OFFSET comment for the full story. A "rest"
# hold using plain ctrl=0 across the board would show neither of these
# poses -- this instead reproduces what run_demo_live.py converges to right
# after zero-pose calibration (shoulder_pitch=shoulder_roll=0 really is the
# hang-down reference for those two, confirmed separately -- only the
# elbow needed correcting).
REST_CTRL = {
    "left_elbow_joint": 1.28,
}


def sweep_value(t, lo, hi, period):
    """Triangle wave 0->1->0 over `period` seconds, mapped to [lo, hi]."""
    phase = (t % period) / period
    frac = 2 * phase if phase < 0.5 else 2 * (1 - phase)
    return lo + frac * (hi - lo)


def geometry_summary(model, data):
    """Numeric stand-in for "does it look right" -- lets a hang/bend
    direction question be answered from this printed text alone, without
    needing to actually watch the interactive window (added 2026-09-02:
    the previous version only printed ctrl targets, which can't be judged
    for correctness without eyes on the live viewer).

    - hang_from_vertical_deg: angle between the shoulder->elbow vector and
      straight down (world -Z, since the frozen mannequin's pelvis sits at
      quat="1 0 0 0" with no rotation, so world axes match its own
      front/up/side directions directly). 0 = upper arm hanging straight
      down; large values mean it's swung away from vertical.
    - elbow_flex_deg: 0 = straight (fully extended), 180 = folded flat back
      onto the upper arm. This is 180 minus the raw angle between the
      shoulder->elbow and elbow->wrist vectors, so it reads as "how much
      it's bent" directly instead of "what's left of straight".
    - wrist_vs_shoulder (front/left/up, meters): wrist position minus
      shoulder position, in world X/Y/Z. +X should mean the wrist is in
      FRONT of the shoulder (toward the mannequin's face) -- a real elbow
      flexion from a hanging arm should push this positive as it curls
      up/forward, never push X negative (that would mean curling backward,
      the exact bug the previous shadow_hand-based rig had).
    """
    shoulder = data.xpos[model.body("left_shoulder_roll_link").id]
    elbow = data.xpos[model.body("left_elbow_link").id]
    wrist = data.xpos[model.body("left_wrist_yaw_link").id]

    upper = elbow - shoulder
    fore = wrist - elbow
    upper_n = upper / np.linalg.norm(upper)
    fore_n = fore / np.linalg.norm(fore)
    raw_angle_deg = np.degrees(np.arccos(np.clip(np.dot(upper_n, fore_n), -1, 1)))
    elbow_flex_deg = 180.0 - raw_angle_deg

    down = np.array([0.0, 0.0, -1.0])
    hang_from_vertical_deg = np.degrees(np.arccos(np.clip(np.dot(upper_n, down), -1, 1)))

    rel = wrist - shoulder
    return (f"hang_from_vertical={hang_from_vertical_deg:5.1f}deg  "
            f"elbow_flex={elbow_flex_deg:5.1f}deg  "
            f"wrist_vs_shoulder(front,left,up)=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f})m")


SYNC_EVERY_N_STEPS = 20  # ~25fps at timestep=0.002s -- see main()'s comment


def hold_rest(model, data, viewer, seconds):
    # Step count derived from sim time, NOT wall-clock time -- see the
    # sweep loop's comment below for why real-time pacing was dropped.
    for name, val in REST_CTRL.items():
        data.ctrl[model.actuator(name).id] = val
    for i in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        if i % SYNC_EVERY_N_STEPS == 0:
            viewer.sync()
            time.sleep(model.opt.timestep * SYNC_EVERY_N_STEPS)


def main():
    if not SCENE_XML.exists():
        raise SystemExit(f"scene not found: {SCENE_XML}")

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Rest pose -- confirm the arm hangs naturally at the mannequin's "
              "left side before the sweeps start.")
        hold_rest(model, data, viewer, PHASE_HOLD_SECONDS)
        print(f"  REST: {geometry_summary(model, data)}")

        for name, (lo, hi), period in PHASES:
            aid = model.actuator(name).id
            print(f"\n[SWEEP] {name}: {lo:.3f} -> {hi:.3f} -> {lo:.3f} rad, "
                  f"{period:.0f}s (sim time) per full cycle -- watch which way it moves.")
            # Driven by a SIM-TIME step count, not wall-clock time: the
            # first version of this script paced itself with
            # `time.sleep(model.opt.timestep)` per step, which only equals
            # real-time if each loop iteration's own Python overhead
            # (prints, the geometry_summary math) takes near-zero time.
            # It doesn't -- confirmed by comparing two runs of the same
            # script, which settled to visibly different rest-pose numbers
            # depending on incidental timing, purely because fewer physics
            # steps had actually run in the same wall-clock 2s hold. Tying
            # the sweep to actual simulated steps instead makes every run
            # of this script reproduce the same numbers regardless of
            # how fast the machine running it happens to be.
            #
            # viewer.sync() itself is NOT called every step -- this scene's
            # full 49-mesh mannequin (vs. the ~15 meshes of the arm-only
            # rig this script was first written against) is expensive
            # enough to render that syncing on every single 2ms physics
            # step made a "16 second" sweep actually take over a minute of
            # real time (measured directly: a 2s rest hold alone exceeded a
            # 75s timeout) -- not a hang, just far more render calls than
            # the log needs. SYNC_EVERY_N_STEPS caps that to a normal frame
            # rate; the physics stepping and printed log are unaffected.
            n_steps = int(period / model.opt.timestep)
            print_every = max(1, n_steps // 12)
            for i in range(n_steps):
                if not viewer.is_running():
                    break
                t = i * model.opt.timestep
                val = sweep_value(t, lo, hi, period)
                data.ctrl[aid] = val
                mujoco.mj_step(model, data)
                if i % SYNC_EVERY_N_STEPS == 0:
                    viewer.sync()
                    time.sleep(model.opt.timestep * SYNC_EVERY_N_STEPS)
                if i % print_every == 0:
                    print(f"  ctrl={val:+.3f}  {geometry_summary(model, data)}")
            print(f"[SWEEP] {name} done, returning to rest")
            hold_rest(model, data, viewer, PHASE_HOLD_SECONDS)

        print("\nAll phases done. Holding final rest pose -- close the viewer window to exit.")
        i = 0
        while viewer.is_running():
            mujoco.mj_step(model, data)
            if i % SYNC_EVERY_N_STEPS == 0:
                viewer.sync()
                time.sleep(model.opt.timestep * SYNC_EVERY_N_STEPS)
            i += 1


if __name__ == "__main__":
    main()

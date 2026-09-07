"""Expands grasp coverage beyond the single front-left reach pose
test_grasp_object.py/test_myoware_grip_replay.py check: sweeps several
distinct reach directions x several grip strengths, all in simulation,
while there's no hardware available to validate the real 6-step task. Each
pose is its own (REACH_CTRL, pedestal_pos, object_pos) triple, found the
same way arm_hand_scene.xml's own front-left spot was: a real settled
mj_step search (grip closed, 800 steps, read the resulting fingertip
centroid) run once offline per pose, not baked into a reusable function
here -- re-run the same search (see arm_hand_scene.xml's pedestal comment
for the method) if a pose needs re-deriving.

This does NOT replace test_grasp_object.py/test_myoware_grip_replay.py --
those stay the fast, single-scenario regression check. This is the wider,
slower sweep for building confidence before real-hardware time, and for
answering "is GRIP_SCALE=0.6 enough everywhere, or did we just get lucky
at the one pose we tested."

2026-09-06/07 RESULT (important, not just a usage note): it was the
latter, though the first pass overstated how bad it was. Initially
front_left looked non-monotonic across grip scale (HELD at 0.30/0.60,
DROPPED at 0.45/0.80/1.00) -- two real bugs in the test itself turned out
to be responsible, both fixed now (see grasp_test_common.py's
POST_LIFT_SETTLE_SECONDS and the `slip` comment in run_grasp_scenario):
the original 1.0s post-lift settle was too short to catch a slow ongoing
fall, and the original `height_drop < 0.05` absolute threshold flagged a
genuinely-held object as DROPPED whenever the commanded lift pose's own
fingertip height sat more than 5cm below the reach pose (confirmed
directly: the fingertip centroid itself drops by about as much as the
object does when this happens -- the object is moving WITH the hand, not
slipping out of it). With both fixed, front_left holds cleanly across
GRIP_SCALE in [0.54, 0.60] and fails outside it (0.50, 0.65) -- a real,
if narrow, working window, not a fragile knife-edge.

What did NOT turn out to be a test bug: the other 4 poses still fail at
every tested grip scale, the same two ways as before: front_center_low/
deep_reach let the object fall all the way through during the grip phase
itself (dist_after ~0.7-0.99m, well before the lift even starts);
front_left_high/mild_adduction never actually pick the object up at all
(dist_after ~0.10-0.23m, roughly constant regardless of grip scale). The
"closed-fist fingertip centroid coincides with the object" search this
project uses to place a pedestal only guarantees geometric reachability,
not that the uniform-curl grip (all 6 GRIP_ACTUATORS scaled by the same
fraction, see run_demo_live.py's GRIP_ACTUATORS comment) actually cups the
object correctly from that approach angle. Per the user's explicit
direction, this is accepted scope, not something to fix with per-finger
control -- the practical takeaway is to keep the real 6-step task's reach
poses close to front_left, not to expect the grasp to generalize.

Usage:
    python3 tools/mujoco_bridge/test_grasp_coverage.py
    python3 tools/mujoco_bridge/test_grasp_coverage.py --grip-scales 0.3,0.45,0.6
"""

import argparse
import sys

import mujoco

import grasp_test_common as gtc

# Each pose is a genuinely different reach direction, verified reachable via
# a real settled mj_step search (grip closed, 800 steps, read the resulting
# thumb/middle/index fingertip centroid) -- same method as
# arm_hand_scene.xml's pedestal comment, just run 4 more times for
# different target directions instead of once. front_left matches the
# scene's own current default pedestal/object position (gtc.DEFAULT_*).
POSES = {
    "front_left": {
        "reach_ctrl": {"left_shoulder_pitch_joint": -1.00, "left_shoulder_roll_joint": 0.80, "left_elbow_joint": 0.90},
        "pedestal_pos": gtc.DEFAULT_PEDESTAL_POS,
        "object_pos": gtc.DEFAULT_OBJECT_POS,
    },
    "front_center_low": {
        "reach_ctrl": {"left_shoulder_pitch_joint": -1.30, "left_shoulder_roll_joint": 0.15, "left_elbow_joint": 0.75},
        "pedestal_pos": (0.061, 0.025, 1.022),
        "object_pos": (0.061, 0.025, 1.077),
    },
    "front_left_high": {
        "reach_ctrl": {"left_shoulder_pitch_joint": -0.55, "left_shoulder_roll_joint": 0.60, "left_elbow_joint": 1.10},
        "pedestal_pos": (-0.034, 0.336, 0.658),
        "object_pos": (-0.034, 0.336, 0.713),
    },
    "mild_adduction": {
        # The one direction this project's own real ROM data says is
        # tight (real adduction ROM ~50deg, see run_demo_live.py's
        # SHOULDER_ROLL_RANGE comment) -- worth testing specifically
        # because it's the harder case, not the easy one.
        "reach_ctrl": {"left_shoulder_pitch_joint": -1.00, "left_shoulder_roll_joint": -0.40, "left_elbow_joint": 0.85},
        "pedestal_pos": (-0.011, -0.143, 0.857),
        "object_pos": (-0.011, -0.143, 0.912),
    },
    "deep_reach": {
        "reach_ctrl": {"left_shoulder_pitch_joint": -1.60, "left_shoulder_roll_joint": 0.35, "left_elbow_joint": 0.95},
        "pedestal_pos": (0.057, 0.081, 1.125),
        "object_pos": (0.057, 0.081, 1.18),
    },
}

# 2026-09-07: narrowed from [0.30,0.45,0.60,0.80,1.00] to bracket the real
# stable-ish window a finer sweep found (see grasp_test_common.py's
# POST_LIFT_SETTLE_SECONDS comment) instead of values spread across the
# whole [0,1] range, most of which fail outright regardless of pose.
DEFAULT_GRIP_SCALES = [0.50, 0.54, 0.57, 0.60, 0.65]
PRODUCTION_GRIP_SCALE = 0.57  # matches run_demo_live.py's current GRIP_SCALE -- the one result that must hold


def grip_ramp(t, ramp_seconds=1.0):
    return min(1.0, t / ramp_seconds)


def run_one(pose, grip_scale):
    model = gtc.load_model(pedestal_pos=pose["pedestal_pos"], object_pos=pose["object_pos"])
    data = mujoco.MjData(model)
    gtc.GRIP_SCALE = grip_scale
    result = gtc.run_grasp_scenario(
        model, data, None, grip_at_t=grip_ramp, grip_phase_seconds=3.0,
        reach_ctrl=pose["reach_ctrl"], verbose=False,
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--grip-scales", default=",".join(str(g) for g in DEFAULT_GRIP_SCALES),
                         help="comma-separated grip scales to sweep")
    parser.add_argument("--poses", default=",".join(POSES.keys()),
                         help="comma-separated pose names to sweep (default: all)")
    args = parser.parse_args()

    grip_scales = [float(g) for g in args.grip_scales.split(",")]
    pose_names = args.poses.split(",")
    for name in pose_names:
        if name not in POSES:
            raise SystemExit(f"unknown pose {name!r} -- choices are {list(POSES.keys())}")

    print(f"Sweeping {len(pose_names)} poses x {len(grip_scales)} grip scales "
          f"({len(pose_names) * len(grip_scales)} runs)...\n")

    rows = []
    for pose_name in pose_names:
        pose = POSES[pose_name]
        for grip_scale in grip_scales:
            result = run_one(pose, grip_scale)
            rows.append((pose_name, grip_scale, result))
            mark = "HELD  " if result["held"] else "DROPPED"
            print(f"  {pose_name:18s} grip_scale={grip_scale:.2f}  {mark}  "
                  f"dist_after={result['object_end_dist']:.3f}m  slip={result['slip']:+.3f}m")

    print(f"\n{'pose':18s} " + "".join(f"{g:>9.2f}" for g in grip_scales))
    for pose_name in pose_names:
        cells = []
        for grip_scale in grip_scales:
            r = next(r for (p, g, r) in rows if p == pose_name and g == grip_scale)
            cells.append(f"{'HELD':>9s}" if r["held"] else f"{'drop':>9s}")
        print(f"{pose_name:18s} " + "".join(cells))

    production_failures = [
        pose_name for (pose_name, grip_scale, r) in rows
        if grip_scale == PRODUCTION_GRIP_SCALE and not r["held"]
    ]
    print(f"\nProduction GRIP_SCALE={PRODUCTION_GRIP_SCALE}: "
          f"{'holds at every tested pose' if not production_failures else 'FAILS at ' + ', '.join(production_failures)}")

    if production_failures:
        sys.exit(1)


if __name__ == "__main__":
    main()

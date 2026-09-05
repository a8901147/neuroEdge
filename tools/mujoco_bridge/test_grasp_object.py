"""Does the current hand rig actually pick up and hold the grasp object, not
just curl fingers near it? test_grip_kinematics.py already confirmed the
hand can open/close through its mapped range with no object involved; this
adds the object back in and checks the physics: reach -> close -> lift, then
see if the ball followed the hand or got left behind.

Uses a hand-authored 0->1 grip ramp (not real EMG) -- see
test_myoware_grip_replay.py for the version driven by decoded MyoWare-style
test data instead. --grip-scale/--ball-radius/--ball-friction let you
experiment with the two mitigations discussed if this comes out DROPPED: a
tighter grip or a smaller/grippier ball.

--headless runs without mujoco.viewer (plain python3, no display needed) --
the printed log (including the final VERDICT line) is the same either way.
Without --headless, must run as `mjpython`, not plain `python3` --
launch_passive raises RuntimeError under plain CPython on macOS.

Usage:
    python3 tools/mujoco_bridge/test_grasp_object.py --headless
    mjpython tools/mujoco_bridge/test_grasp_object.py
    mjpython tools/mujoco_bridge/test_grasp_object.py --grip-scale 1.0 --ball-radius 0.018
"""

import argparse

import grasp_test_common as gtc


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    gtc.add_common_args(parser)
    parser.add_argument("--grip-ramp-seconds", type=float, default=1.0,
                         help="how long the hand-authored ramp takes to go 0->1")
    parser.add_argument("--grip-phase-seconds", type=float, default=3.0,
                         help="total time driving grip before the lift phase starts "
                              "(must be >= --grip-ramp-seconds so grip actually reaches 1.0 and holds)")
    args = parser.parse_args()

    def grip_ramp(t):
        return min(1.0, t / args.grip_ramp_seconds)

    return gtc.run_with_viewer_or_headless(args, grip_at_t=grip_ramp,
                                            grip_phase_seconds=args.grip_phase_seconds)


if __name__ == "__main__":
    main()

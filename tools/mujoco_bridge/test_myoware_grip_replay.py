"""Drives the grasp scenario from decoded MyoWare-style EMG test data instead
of a hand-authored grip ramp (that's test_grasp_object.py). Reuses
data/wearable_1emg_12imu.csv's emg0 column -- an existing, already-
established test fixture in this repo (tools/generate_sample_data.py:
"NOT real Ninapro... produces signals with the same statistical shape
(rest/contraction bursts for EMG)"), not a new signal invented for this
script. Its single contraction burst (~0.8s-1.6s of the 2s recording) is the
"squeeze" that should trigger a grasp.

The EMG -> grip decode is a faithful Python port of the actual production
logic (include/edgeneuro/control/grip_state_machine.hpp's GripStateMachine +
slew_rate_limiter.hpp's SlewRateLimiter), run with the SAME constants
src/mujoco_bridge_demo.cpp uses for this exact CSV (kGripThreshold=0.15,
kOnDuration=kOffDuration=0.1, kSlewRate=1.5, kDt=0.001) -- not reimplemented
loosely, so this test exercises the real decision logic's real behavior on
this data, just without needing the C++ binary built or a subprocess.

--headless runs without mujoco.viewer (plain python3, no display needed) --
the printed log (including the final VERDICT line) is the same either way.
Without --headless, must run as `mjpython`, not plain `python3` --
launch_passive raises RuntimeError under plain CPython on macOS.

Usage:
    python3 tools/mujoco_bridge/test_myoware_grip_replay.py --headless
    mjpython tools/mujoco_bridge/test_myoware_grip_replay.py
"""

import argparse
import csv

import grasp_test_common as gtc

REPO_ROOT = gtc.REPO_ROOT
EMG_CSV = REPO_ROOT / "data" / "wearable_1emg_12imu.csv"

# Must match src/mujoco_bridge_demo.cpp's constants exactly -- see that
# file's own kGripThreshold comment for why 0.15 (not the real hardware's
# calibrated ADC-count threshold) is correct for THIS dataset's 0-1 scale.
GRIP_THRESHOLD = 0.15
ON_DURATION = 0.1
OFF_DURATION = 0.1
SLEW_RATE = 1.5
DT = 0.001  # 1kHz, matches generate_sample_data.py's fs_hz and mujoco_bridge_demo.cpp's kDt


class GripStateMachine:
    """Line-for-line port of grip_state_machine.hpp's GripStateMachine
    (threshold + on/off hysteresis debounce) -- same fields, same update()
    logic, just Python. See that header for the full rationale comment."""

    def __init__(self, threshold, on_duration, off_duration):
        self.threshold = threshold
        self.on_duration = on_duration
        self.off_duration = off_duration
        self.above_time = 0.0
        self.below_time = 0.0
        self.is_gripping = False

    def update(self, envelope, dt):
        if envelope > self.threshold:
            self.above_time += dt
            self.below_time = 0.0
        else:
            self.below_time += dt
            self.above_time = 0.0

        if not self.is_gripping and self.above_time >= self.on_duration:
            self.is_gripping = True
        elif self.is_gripping and self.below_time >= self.off_duration:
            self.is_gripping = False


class SlewRateLimiter:
    """Line-for-line port of slew_rate_limiter.hpp's SlewRateLimiter."""

    def __init__(self, max_rate, initial=0.0):
        self.max_rate = max_rate
        self.current = initial

    def update(self, target, dt):
        max_step = self.max_rate * dt
        delta = target - self.current
        if delta > max_step:
            self.current += max_step
        elif delta < -max_step:
            self.current -= max_step
        else:
            self.current = target
        return self.current


def decode_grip_trajectory(emg_samples, dt):
    """Runs the real decode logic once over the whole EMG series, returns a
    list of (t, grip) -- computed upfront (not tick-by-tick against MuJoCo's
    own physics timestep) since the two have unrelated sample rates (EMG at
    1kHz here vs. MuJoCo's 2ms physics step) and the decode has no
    dependency on MuJoCo state."""
    grip_machine = GripStateMachine(GRIP_THRESHOLD, ON_DURATION, OFF_DURATION)
    slew = SlewRateLimiter(SLEW_RATE)
    trajectory = []
    for i, envelope in enumerate(emg_samples):
        t = i * dt
        grip_machine.update(envelope, dt)
        grip = slew.update(1.0 if grip_machine.is_gripping else 0.0, dt)
        trajectory.append((t, grip))
    return trajectory


def make_grip_at_t(trajectory):
    """Nearest-earlier-sample lookup -- returns the last trajectory value if
    t runs past the end of the recorded EMG data (only matters if a caller
    asks for a grip_phase_seconds longer than the CSV covers)."""
    def grip_at_t(t):
        idx = int(t / DT)
        idx = max(0, min(idx, len(trajectory) - 1))
        return trajectory[idx][1]
    return grip_at_t


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    gtc.add_common_args(parser)
    parser.add_argument(
        "--grip-phase-seconds", type=float, default=1.65,
        help="cut over to the lift phase at this sim-time point in the EMG replay -- "
             "default 1.65s lands just after the slew-limited ramp finishes closing "
             "(threshold crossing ~0.8s + 0.1s on-duration + 0.667s slew ramp ~= 1.57s) "
             "but before the contraction burst ends at 1.6s and release starts ramping "
             "grip back down",
    )
    args = parser.parse_args()

    if not EMG_CSV.exists():
        raise SystemExit(f"EMG test data not found: {EMG_CSV} -- run tools/generate_sample_data.py first")

    with open(EMG_CSV) as f:
        rows = list(csv.DictReader(f))
    emg_samples = [float(row["emg0"]) for row in rows]
    print(f"Loaded {len(emg_samples)} EMG samples from {EMG_CSV} "
          f"({len(emg_samples) * DT:.2f}s @ {1.0/DT:.0f}Hz)")

    trajectory = decode_grip_trajectory(emg_samples, DT)
    n_gripping_transitions = sum(
        1 for i in range(1, len(trajectory))
        if (trajectory[i][1] > 0) != (trajectory[i - 1][1] > 0)
    )
    print(f"Decoded grip trajectory: starts at {trajectory[0][1]:.3f}, "
          f"peak={max(g for _, g in trajectory):.3f}, "
          f"ends at {trajectory[-1][1]:.3f}, "
          f"zero-crossings={n_gripping_transitions}")

    grip_at_t = make_grip_at_t(trajectory)
    return gtc.run_with_viewer_or_headless(args, grip_at_t=grip_at_t,
                                            grip_phase_seconds=args.grip_phase_seconds)

    return result


if __name__ == "__main__":
    main()

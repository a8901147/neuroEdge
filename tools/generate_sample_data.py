#!/usr/bin/env python3
"""Generates synthetic CSV fixtures for EdgeNeuro Phase 1.

These are NOT real Ninapro recordings — Ninapro requires a data-use
agreement and is too large to vendor into the repo. This script produces
signals with the same statistical shape (rest/contraction bursts for EMG,
smooth rotation for IMU) so CsvSignalProvider and the demo oscilloscope can
be exercised end-to-end without any external dataset dependency. Swap in a
real Ninapro CSV export later by matching the same column layout.
"""
import math
import random

random.seed(42)

def wearable_fusion_csv(path: str, num_samples: int = 2000, fs_hz: float = 1000.0) -> None:
    """1-ch EMG (grasp intent) + 6-axis IMU (wrist/forearm pose), 1kHz."""
    with open(path, "w") as f:
        f.write("emg0,imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz\n")
        for i in range(num_samples):
            t = i / fs_hz
            # Alternate rest (~40% MVC noise floor) and contraction bursts every ~0.8s.
            in_burst = (i // int(0.8 * fs_hz)) % 2 == 1
            base = 0.6 if in_burst else 0.03
            emg = base * (0.5 + random.random()) + random.gauss(0, 0.02)

            # Slow forearm rotation + gyro noise, orientation loosely tracks the burst state.
            angle = 0.3 * math.sin(2 * math.pi * 0.25 * t)
            ax = math.sin(angle) + random.gauss(0, 0.01)
            ay = math.cos(angle) * 0.2 + random.gauss(0, 0.01)
            az = 9.81 + random.gauss(0, 0.02)
            gx = 0.3 * math.cos(2 * math.pi * 0.25 * t) + random.gauss(0, 0.02)
            gy = random.gauss(0, 0.02)
            gz = random.gauss(0, 0.02)

            f.write(f"{emg:.5f},{ax:.5f},{ay:.5f},{az:.5f},{gx:.5f},{gy:.5f},{gz:.5f}\n")


def wearable_arm_fusion_csv(path: str, num_samples: int = 2000, fs_hz: float = 1000.0) -> None:
    """1-ch EMG + 2x 6-axis IMU (upper arm, forearm), 1kHz.

    Phase 2 iteration 2 fixture: whole-arm reach (shoulder pitch+roll from
    IMU#1, elbow flexion from IMU#2 relative to IMU#1) + EMG-driven grasp.
    Matches the confirmed real Phase 3 sensor budget (2x MPU6050, one per
    limb segment -- see PRD.md Section 3).

    The forearm IMU's simulated orientation is built as the upper-arm's
    orientation PLUS a separately-varying elbow-bend term (not an
    independent random signal), so the C++ bridge's relative-pitch elbow
    math (forearm_pitch - shoulder_pitch) corresponds to a physically
    coherent motion instead of two unrelated angles. The reach/elbow-bend
    envelope reuses the same 0.8s alternation block boundaries as the EMG
    burst schedule, so the grip-closing window and the arm's
    extended-reach phase are synchronized by construction.

    ax/ay/az per IMU are generated as the algebraic inverse of
    ComplementaryFilter's own atan2-based accel_roll_angle/accel_pitch_angle
    (ax=-g*sin(pitch), ay=g*cos(pitch)*sin(roll), az=g*cos(pitch)*cos(roll)),
    so the signal round-trips through the real filter back to the intended
    angle rather than approximating it.

    Shoulder-pitch/elbow-bend REST/REACH values and the asymmetric rise/fall
    timing below were tuned by directly stepping the target MuJoCo arm+hand
    model (tools/mujoco_bridge/arm_hand_scene.xml) through full reach/grip/
    retract cycles and reading where the hand's grasp_site and the grasp
    object actually end up -- not guessed from kinematics alone.
    """
    block_samples = int(0.8 * fs_hz)
    reach = 0.0
    # Asymmetric: reaching out is slow (time constant ~0.33s) so the arm
    # doesn't slam into the grasp object on approach (verified empirically --
    # a faster ~0.1s constant launched the object off its pedestal).
    # Retracting is much faster (~50ms) so the arm gets back near the
    # pedestal well before GripStateMachine/SlewRateLimiter's own release
    # (which takes ~0.5-0.7s in the C++ bridge) has fully opened the fingers
    # -- without this asymmetry, the fingers were still opening while the
    # arm was still mid-air on the way back, dropping the object partway
    # through the retract instead of setting it back down near the pedestal.
    reach_alpha_rise = 0.003
    reach_alpha_fall = 0.003

    SHOULDER_PITCH_REST, SHOULDER_PITCH_REACH = -0.15, 0.0
    ELBOW_BEND_REST, ELBOW_BEND_REACH = 0.6, 0.05
    SHOULDER_ROLL_AMPLITUDE = 0.3

    prev_shoulder_pitch = SHOULDER_PITCH_REST
    prev_shoulder_roll = 0.0
    prev_forearm_pitch = SHOULDER_PITCH_REST + ELBOW_BEND_REST
    g = 9.81

    with open(path, "w") as f:
        f.write("emg0,imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,"
                 "imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz\n")
        for i in range(num_samples):
            t = i / fs_hz
            in_burst = (i // block_samples) % 2 == 1
            base = 0.6 if in_burst else 0.03
            emg = base * (0.5 + random.random()) + random.gauss(0, 0.02)

            reach_target = 1.0 if in_burst else 0.0
            reach_alpha = reach_alpha_rise if reach_target > reach else reach_alpha_fall
            reach += reach_alpha * (reach_target - reach)
            shoulder_pitch = SHOULDER_PITCH_REST + reach * (SHOULDER_PITCH_REACH - SHOULDER_PITCH_REST)
            shoulder_roll = SHOULDER_ROLL_AMPLITUDE * math.sin(2 * math.pi * 0.1 * t)
            elbow_bend = ELBOW_BEND_REST + reach * (ELBOW_BEND_REACH - ELBOW_BEND_REST)
            forearm_pitch = shoulder_pitch + elbow_bend
            forearm_roll = shoulder_roll  # single-plane simplification: elbow adds no independent roll

            ax1 = -g * math.sin(shoulder_pitch) + random.gauss(0, 0.01)
            ay1 = g * math.cos(shoulder_pitch) * math.sin(shoulder_roll) + random.gauss(0, 0.01)
            az1 = g * math.cos(shoulder_pitch) * math.cos(shoulder_roll) + random.gauss(0, 0.02)
            gx1 = (shoulder_roll - prev_shoulder_roll) * fs_hz + random.gauss(0, 0.02)
            gy1 = (shoulder_pitch - prev_shoulder_pitch) * fs_hz + random.gauss(0, 0.02)
            gz1 = random.gauss(0, 0.02)  # yaw unobservable, no magnetometer -- pure noise

            ax2 = -g * math.sin(forearm_pitch) + random.gauss(0, 0.01)
            ay2 = g * math.cos(forearm_pitch) * math.sin(forearm_roll) + random.gauss(0, 0.01)
            az2 = g * math.cos(forearm_pitch) * math.cos(forearm_roll) + random.gauss(0, 0.02)
            gx2 = (forearm_roll - prev_shoulder_roll) * fs_hz + random.gauss(0, 0.02)
            gy2 = (forearm_pitch - prev_forearm_pitch) * fs_hz + random.gauss(0, 0.02)
            gz2 = random.gauss(0, 0.02)

            prev_shoulder_pitch, prev_shoulder_roll, prev_forearm_pitch = shoulder_pitch, shoulder_roll, forearm_pitch

            f.write(f"{emg:.5f},{ax1:.5f},{ay1:.5f},{az1:.5f},{gx1:.5f},{gy1:.5f},{gz1:.5f},"
                     f"{ax2:.5f},{ay2:.5f},{az2:.5f},{gx2:.5f},{gy2:.5f},{gz2:.5f}\n")


def hd_semg_csv(path: str, num_channels: int = 32, num_samples: int = 2000, fs_hz: float = 1000.0) -> None:
    """32-channel HD-sEMG stress-test fixture: independent bursty channels."""
    with open(path, "w") as f:
        f.write(",".join(f"emg{c}" for c in range(num_channels)) + "\n")
        phase = [random.uniform(0, 2 * math.pi) for _ in range(num_channels)]
        for i in range(num_samples):
            t = i / fs_hz
            row = []
            for c in range(num_channels):
                in_burst = math.sin(2 * math.pi * 0.3 * t + phase[c]) > 0.3
                base = 0.5 if in_burst else 0.03
                row.append(base * (0.5 + random.random()) + random.gauss(0, 0.02))
            f.write(",".join(f"{v:.5f}" for v in row) + "\n")


if __name__ == "__main__":
    import pathlib
    data_dir = pathlib.Path(__file__).resolve().parent.parent / "data"
    data_dir.mkdir(exist_ok=True)
    wearable_fusion_csv(str(data_dir / "wearable_1emg_6imu.csv"))
    wearable_arm_fusion_csv(str(data_dir / "wearable_1emg_12imu.csv"))
    hd_semg_csv(str(data_dir / "hd_semg_32ch.csv"))
    print(f"Wrote sample data to {data_dir}")

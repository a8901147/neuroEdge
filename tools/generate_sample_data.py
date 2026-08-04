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
    hd_semg_csv(str(data_dir / "hd_semg_32ch.csv"))
    print(f"Wrote sample data to {data_dir}")

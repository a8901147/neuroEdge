#!/usr/bin/env python3
"""Converts EMG-EPN-612 (Zenodo record 4421500) per-user JSON recordings into
the flat CSV layout CsvSignalProvider expects: EmgChannels columns first,
then ImuChannels columns, one row per EMG sample.

Confirmed real schema (data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json,
inspected directly — not guessed):

    sample = {
        "emg": {"ch1": [...], ..., "ch8": [...]},        # 8 x N_emg raw ints
        "accelerometer": {"x": [...], "y": [...], "z": [...]},   # 3 x N_imu
        "gyroscope":      {"x": [...], "y": [...], "z": [...]},  # 3 x N_imu
        "quaternion":      {"w": [...], "x": [...], "y": [...], "z": [...]},  # 4 x N_imu
        "gestureName": str,
        "myoDetection": [...],                    # per-IMU-sample flag, unused here
        "startPointforGestureExecution": int,      # unused here
    }

N_emg (992 in user1's first sample) and N_imu (249) are NOT a clean integer
ratio (992/249 ~= 3.98, not exactly 4) — EMG (~200Hz ADC) and IMU (~50Hz,
I2C-polled) are genuinely independent streams on this real Myo hardware,
confirming that's the realistic embedded-systems case, not a synthetic
simplification. The fix is NOT to change the C++ engine: resample the
slower IMU stream up to the EMG sample count with zero-order hold (repeat
the last known IMU reading until proportionally the next one would arrive),
by length ratio rather than an assumed fixed Hz value. Real Phase 3
firmware would do the same thing in its main loop: run at the EMG ADC's
rate, only refresh the IMU reading every Nth tick.

Usage:
    python3 tools/convert_epn612.py --convert \
        data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json \
        --out data/epn612_user1.csv

    python3 tools/convert_epn612.py --inspect \
        data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json
"""
import argparse
import json

NUM_EMG_CHANNELS = 8
EMG_NORMALIZATION = 128.0  # matches the convention used by other EPN-612 loaders

# (output_column_prefix, top_level_key, ordered_sub_keys)
IMU_FIELDS = [
    ("acc", "accelerometer", ["x", "y", "z"]),
    ("gyro", "gyroscope", ["x", "y", "z"]),
    ("quat", "quaternion", ["w", "x", "y", "z"]),
]
NUM_IMU_COLUMNS = sum(len(sub_keys) for _, _, sub_keys in IMU_FIELDS)


def _describe(value, depth: int = 0) -> str:
    indent = "  " * depth
    if isinstance(value, dict):
        lines = [f"dict with {len(value)} keys: {sorted(value.keys())}"]
        for k in sorted(value.keys()):
            lines.append(f"{indent}  .{k} -> {_describe(value[k], depth + 1)}")
        return "\n".join(lines)
    if isinstance(value, list):
        if not value:
            return "empty list"
        return f"list of length {len(value)}, element[0] = {_describe(value[0], depth + 1)}"
    return f"{type(value).__name__} = {value!r}"


def inspect(path: str) -> None:
    """Prints the full real JSON structure of one sample. Useful for a
    different user/session file, or if a future EPN-612 release changes
    the schema documented above."""
    with open(path) as f:
        data = json.load(f)

    for split in ("trainingSamples", "testingSamples"):
        samples = data.get(split, {})
        if not samples:
            continue
        first_id = next(iter(samples))
        sample = samples[first_id]
        print(f"=== {split} -> sample '{first_id}' ===")
        for key in sorted(sample.keys()):
            print(f"'{key}': {_describe(sample[key])}")
        return  # one sample is enough to see the schema
    print("No trainingSamples/testingSamples found — top-level keys were:", sorted(data.keys()))


def extract_imu_rows(sample: dict):
    """Transposes the column-oriented accelerometer/gyroscope/quaternion
    dicts into a list of per-instant 10-value tuples
    (acc_x,acc_y,acc_z, gyro_x,gyro_y,gyro_z, quat_w,quat_x,quat_y,quat_z)."""
    per_field_series = []
    for _, top_key, sub_keys in IMU_FIELDS:
        block = sample[top_key]  # KeyError here means the real schema changed — fail loudly
        per_field_series.extend(block[k] for k in sub_keys)

    num_imu_samples = len(per_field_series[0])
    return [tuple(series[t] for series in per_field_series) for t in range(num_imu_samples)]


def zero_order_hold_resample(imu_rows, num_emg_samples: int):
    """Maps each EMG sample index to the IMU sample that would be current
    at the same proportional position in the recording (by length ratio,
    not an assumed fixed Hz — N_emg/N_imu isn't a clean integer here)."""
    num_imu_samples = len(imu_rows)
    return [imu_rows[min(i * num_imu_samples // num_emg_samples, num_imu_samples - 1)]
            for i in range(num_emg_samples)]


def convert(path: str, out_path: str) -> None:
    with open(path) as f:
        data = json.load(f)

    rows_written = 0
    with open(out_path, "w") as out:
        header = [f"emg{c}" for c in range(NUM_EMG_CHANNELS)] + \
                 [f"{prefix}_{sub}" for prefix, _, sub_keys in IMU_FIELDS for sub in sub_keys]
        out.write(",".join(header) + "\n")

        for split in ("trainingSamples", "testingSamples"):
            for sample in data.get(split, {}).values():
                ch_keys = sorted(sample["emg"].keys())  # 'ch1'..'ch8', lexicographic == numeric here
                if len(ch_keys) != NUM_EMG_CHANNELS:
                    raise ValueError(f"expected {NUM_EMG_CHANNELS} EMG channels, got {ch_keys}")
                emg_series = [sample["emg"][k] for k in ch_keys]
                num_emg_samples = len(emg_series[0])

                imu_rows = extract_imu_rows(sample)
                imu_aligned = zero_order_hold_resample(imu_rows, num_emg_samples)

                for i in range(num_emg_samples):
                    emg_vals = [emg_series[c][i] / EMG_NORMALIZATION for c in range(NUM_EMG_CHANNELS)]
                    row_vals = emg_vals + list(imu_aligned[i])
                    out.write(",".join(f"{v:.6f}" for v in row_vals) + "\n")
                    rows_written += 1

    print(f"Wrote {rows_written} rows to {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--inspect", metavar="JSON_FILE", help="print real JSON structure of one file")
    group.add_argument("--convert", metavar="JSON_FILE", help="convert one user's JSON file to CSV")
    parser.add_argument("--out", metavar="CSV_FILE", help="output path (required with --convert)")
    args = parser.parse_args()

    if args.inspect:
        inspect(args.inspect)
    else:
        if not args.out:
            parser.error("--convert requires --out")
        convert(args.convert, args.out)

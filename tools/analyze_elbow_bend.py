#!/usr/bin/env python3
"""How the firmware's elbow-bend reading changes with arm pose in recorded arm motion.

The firmware takes the elbow bend as the angle between the two raw gravity vectors, each in its own sensor's frame
(mearm_input_filter.hpp: elbow_bend_between). That equals the elbow angle only when the two sensor frames coincide with
the arm straight and the elbow's axis is horizontal. A mounting misalignment adds an offset that changes with the arm's
pose; the calibration's zero_elbow removes it at one pose only.

For each "raise_" phase of a data/arm_motion_*.json recording (each starts from a hanging arm), this prints the mean
reading per 15-degree band of upper-arm tilt from the phase's first sample, and the change from hanging to the highest
band. The recordings do not log the true elbow angle, so a change mixes the offset drift with any real elbow motion.

    python3 tools/analyze_elbow_bend.py                       # every data/arm_motion_*.json
    python3 tools/analyze_elbow_bend.py data/arm_motion_20261004-021240.json
"""
import argparse
import json
import math
import sys
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parents[1] / "data"
BAND_DEG = 15
MIN_SAMPLES_PER_BAND = 5


def angle_deg(a, b):
    """Angle between two vectors in degrees -- elbow_bend_between() in mearm_input_filter.hpp, in degrees."""
    ma = math.sqrt(sum(x * x for x in a))
    mb = math.sqrt(sum(x * x for x in b))
    if not (ma > 0.1 and mb > 0.1):
        return 0.0
    c = sum(x * y for x, y in zip(a, b)) / (ma * mb)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def bend_by_tilt(samples):
    """{band start (deg): mean elbow-bend reading (deg)} over upper-arm tilt from the first sample, for bands with at
    least MIN_SAMPLES_PER_BAND samples."""
    if not samples:
        return {}
    hang = samples[0]["upper_arm"]
    bands = {}
    for s in samples:
        tilt = angle_deg(hang, s["upper_arm"])
        bands.setdefault(int(tilt // BAND_DEG) * BAND_DEG, []).append(angle_deg(s["upper_arm"], s["forearm"]))
    return {k: sum(v) / len(v) for k, v in sorted(bands.items()) if len(v) >= MIN_SAMPLES_PER_BAND}


def report(path):
    phases = json.loads(Path(path).read_text())["phases"]
    lines = [f"{Path(path).name}"]
    for name, samples in phases.items():
        if not name.startswith("raise_"):
            continue
        bands = bend_by_tilt(samples)
        if len(bands) < 2:
            continue
        low, high = min(bands), max(bands)
        cells = "  ".join(f"{k}-{k + BAND_DEG}: {v:5.1f}" for k, v in bands.items())
        lines.append(f"  {name:18s} {cells}  | change {bands[high] - bands[low]:+5.1f} deg")
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("files", nargs="*", help="recordings (default: every data/arm_motion_*.json)")
    args = parser.parse_args(argv)
    files = args.files or sorted(str(p) for p in DATA_DIR.glob("arm_motion_*.json"))
    found = False
    for f in files:
        lines = report(f)
        if len(lines) > 1:
            found = True
            print("\n".join(lines))
    if not found:
        print("No recording with raise_ phases found.", file=sys.stderr)
        return 1
    print("Columns: upper-arm tilt band (deg from the phase's first, hanging sample): mean elbow-bend reading (deg).")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Lists every entry run_demo_live.py's calibrate_emg_threshold() has
logged to emg_calibration_logs/ (see EMG_CALIBRATION_LOG_DIR's own comment
in run_demo_live.py -- this is the long-term "is mean+K*std actually the
right algorithm" observation the user asked for 2026-09-19, not something
this script itself judges). One row per real calibration session: when,
which EMG_THRESHOLD_K/git commit produced it, the computed threshold and
margin, and whether it was auto-flagged SUSPECT (contracted_mean never
cleared its own threshold) purely from the filename -- no need to open
every JSON file by hand to decide what's worth a closer look.

Usage:
    python3 tools/mujoco_bridge/summarize_emg_calibration_logs.py
    python3 tools/mujoco_bridge/summarize_emg_calibration_logs.py --suspect-only
"""

import argparse
import json
import sys
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent / "emg_calibration_logs"


def load_entries(log_dir):
    """Returns a list of (path, data) for every *.json in log_dir, sorted
    by filename (== chronological, since the timestamp is the filename
    prefix) -- skips and warns on a file that doesn't parse as JSON rather
    than crashing the whole summary over one corrupt entry."""
    entries = []
    for path in sorted(log_dir.glob("*.json")):
        try:
            with open(path) as f:
                entries.append((path, json.load(f)))
        except (json.JSONDecodeError, OSError) as e:
            print(f"[skip] {path.name}: {e}", file=sys.stderr)
    return entries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--suspect-only", action="store_true", help="only list SUSPECT-flagged entries")
    parser.add_argument("--log-dir", type=Path, default=LOG_DIR)
    args = parser.parse_args()

    if not args.log_dir.exists():
        print(f"{args.log_dir} doesn't exist yet -- no calibration sessions logged so far "
              f"(run_demo_live.py's calibrate_emg_threshold() creates it on first real session).")
        return

    entries = load_entries(args.log_dir)
    if args.suspect_only:
        entries = [(p, d) for (p, d) in entries if d.get("suspect")]

    if not entries:
        print("no matching log entries.")
        return

    header = f"{'timestamp':17s} {'commit':8s} {'K':>6s} {'threshold':>9s} {'contracted_mean':>15s} {'margin':>8s}  flag"
    print(header)
    print("-" * len(header))
    for path, d in entries:
        margin = d["contracted_mean"] - d["threshold"]
        flag = "SUSPECT" if d.get("suspect") else ""
        print(f"{d.get('timestamp', '?'):17s} {str(d.get('git_commit') or '?'):8s} "
              f"{d.get('emg_threshold_k', float('nan')):6.1f} {d.get('threshold', float('nan')):9.0f} "
              f"{d.get('contracted_mean', float('nan')):15.0f} {margin:8.0f}  {flag}")

    n_suspect = sum(1 for _, d in entries if d.get("suspect"))
    print(f"\n{len(entries)} entries ({n_suspect} flagged SUSPECT).")


if __name__ == "__main__":
    main()

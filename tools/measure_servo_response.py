#!/usr/bin/env python3
"""Interactive: what the board REALLY sends to the four MEArm servos, phase by phase.

Reads TIM3->CCR1..4 (the pulse widths on PA6/PA7/PB0/PB1: base, shoulder, elbow, claw) over SWD with the ST-Link,
~800 times a second, while you hold still or move. Nothing is flashed and the firmware keeps running. This measures the
servo signal itself -- not the MuJoCo preview, which is a different code path (2026-10-03).

Each phase starts only when you press Enter (3-second countdown) and can be redone. Per phase and servo it reports the
range, the spread and std while you hold still (jitter), how big each step is, and how often a step hits the ramp's
speed limit (~60 us per 10 ms update). Results go to a NEW data/servo_response_<timestamp>.json.

Needs: phase3_control_loop with servos ON running, the ST-Link attached, servo power on (servos move as usual).

    python3 tools/measure_servo_response.py
"""
import json
import re
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OPENOCD_CFG = REPO / "firmware" / "openocd.cfg"
DATA_DIR = REPO / "data"
CCR1_ADDR = 0x40000434            # TIM3 base 0x40000400 + CCR1 offset 0x34 (CCR2..4 follow, 4 bytes apart)
RATE_LIMIT_STEP_US = 55           # the ramp allows ~60 us per 10 ms servo update; a step this big is AT the limit
SERVOS = ("base", "shoulder", "elbow", "claw")

# (name, label, seconds, what to do) -- the still holds are where the base was seen to swing (2026-10-03: upper arm
# nearly hanging, base 1446..2500), then the demo task itself, then free motion as a check.
PHASES = [
    ("hold_hang", "Still: arm hanging", 10.0, "Let the arm hang naturally and do not move at all."),
    ("hold_half", "Still: raised halfway forward", 10.0, "Raise the arm forward to about 45 degrees and keep still."),
    ("hold_forward", "Still: forward, level", 10.0, "Hold the arm straight forward, level with the floor, and keep still."),
    ("task", "Demo task", 25.0,
     "Do the demo once: hang -> forward -> swing left -> grip -> lift -> swing right -> place, pausing 2 s at each pose."),
    ("check_free", "Check: free movement", 15.0, "(not in the task) Move freely, fast and slow."),
]


def tcl_script(out_path, seconds):
    us = int(round(seconds * 1e6))
    return (f"init; set f [open {out_path} w]; set t0 [clock microseconds]; "
            f"while {{[clock microseconds] - $t0 < {us}}} {{ set v [read_memory 0x{CCR1_ADDR:08x} 32 4]; "
            f"puts $f \"[expr {{[clock microseconds] - $t0}}] $v\" }}; close $f; exit")


def parse_samples(text):
    out = []
    for line in text.splitlines():
        m = re.fullmatch(r"(\d+) (0x[0-9a-fA-F]+) (0x[0-9a-fA-F]+) (0x[0-9a-fA-F]+) (0x[0-9a-fA-F]+)", line.strip())
        if m:
            out.append((int(m.group(1)) / 1e6, *(int(g, 16) for g in m.groups()[1:])))
    return out


def record_swd(seconds):
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "ccr.txt"
        subprocess.run(["openocd", "-f", str(OPENOCD_CFG), "-c", tcl_script(path, seconds)],
                       capture_output=True, text=True, timeout=seconds + 30)
        return parse_samples(path.read_text()) if path.exists() else []


def channel_stats(samples, ch):
    vals = [s[ch + 1] for s in samples]
    if not vals:
        return {"n": 0, "min": None, "max": None, "spread": None, "std": None, "changes": 0, "max_step": 0,
                "at_rate_limit": 0}
    steps = [abs(b - a) for a, b in zip(vals, vals[1:]) if b != a]
    return {"n": len(vals), "min": min(vals), "max": max(vals), "spread": max(vals) - min(vals),
            "std": statistics.pstdev(vals), "changes": len(steps), "max_step": max(steps, default=0),
            "at_rate_limit": sum(1 for s in steps if s >= RATE_LIMIT_STEP_US)}


def summary_lines(phases):
    lines = []
    for name, samples in phases.items():
        label = next((p[1] for p in PHASES if p[0] == name), name)
        dur = samples[-1][0] if samples else 0.0
        lines.append(f"\n[{label}] {len(samples)} samples / {dur:.1f} s")
        for ch, servo in enumerate(SERVOS):
            st = channel_stats(samples, ch)
            if not st["n"]:
                lines.append(f"  {servo}: no data")
                continue
            lines.append(f"  {servo}: {st['min']}-{st['max']} µs (spread {st['spread']}, std {st['std']:.1f}), "
                         f"largest step {st['max_step']} µs, at the rate limit {st['at_rate_limit']} times")
    return lines


def run_protocol(record, input_fn=input, out=print, sleep=time.sleep, phases=PHASES):
    results = {}
    for i, (name, label, seconds, what) in enumerate(phases, 1):
        while True:
            out(f"\n[{i}/{len(phases)}] {label} ({seconds:.0f} s)\n   {what}")
            input_fn("   Press Enter to start: ")
            for c in (3, 2, 1):
                out(f"   {c}…")
                sleep(1.0)
            out("   recording...")
            samples = record(seconds)
            out(f"   done: {len(samples)} samples")
            if input_fn("   Keep this phase? Enter=keep  r=re-record: ").strip().lower() == "r":
                continue
            results[name] = samples
            break
    return results


def save(phases, data_dir=DATA_DIR, name=None):
    data_dir = Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    path = data_dir / (name or time.strftime("servo_response_%Y%m%d-%H%M%S.json"))
    if path.exists():
        raise FileExistsError(f"{path} already exists and will not be overwritten")
    path.write_text(json.dumps({"measured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                                "columns": ["t_s", "base_us", "shoulder_us", "elbow_us", "claw_us"],
                                "phases": {k: [list(s) for s in v] for k, v in phases.items()}}, indent=1) + "\n")
    return path


def main():
    print(__doc__.split("\n\n")[0])
    print("\nThe servo supply must be on (the servos follow you as usual). The board must run the servo build of "
          "phase3_control_loop, with the ST-Link connected.")
    test = record_swd(0.5)
    if not test:
        sys.exit("Cannot read the TIM3 registers. Is the ST-Link connected and the board running? "
                 "(check with check_hardware_ready.py --boot-check)")
    print(f"ST-Link OK ({len(test)} samples in 0.5 s).")
    phases = run_protocol(record_swd)
    path = save(phases)
    print("\n".join(summary_lines(phases)))
    print(f"\nSaved to {path}")


if __name__ == "__main__":
    main()

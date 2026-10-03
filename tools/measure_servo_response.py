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
SERVOS = ("底座", "肩膀", "手肘", "夾爪")

# (name, label, seconds, what to do) -- the still holds are where the base was seen to swing (2026-10-03: upper arm
# nearly hanging, base 1446..2500), then the demo task itself, then free motion as a check.
PHASES = [
    ("hold_hang", "靜止：手臂垂下", 10.0, "手臂自然垂下，完全不要動。"),
    ("hold_half", "靜止：往前抬一半", 10.0, "手臂往前抬到大約 45 度，停住不動。"),
    ("hold_forward", "靜止：往前伸平", 10.0, "手臂往前伸直、和地面平行，停住不動。"),
    ("task", "demo 任務", 25.0, "照 demo 做一遍：垂下→往前伸平→左擺→握拳→抬起→右擺→放下，每個姿勢停 2 秒。"),
    ("check_free", "驗證：自由動作", 15.0, "（不在任務裡）隨意地動，快慢都有。"),
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
        lines.append(f"\n【{label}】{len(samples)} 筆 / {dur:.1f} 秒")
        for ch, servo in enumerate(SERVOS):
            st = channel_stats(samples, ch)
            if not st["n"]:
                lines.append(f"  {servo}：沒有資料")
                continue
            lines.append(f"  {servo}：{st['min']}~{st['max']} µs（範圍 {st['spread']}，標準差 {st['std']:.1f}），"
                         f"每步最大 {st['max_step']} µs，撞到速度上限 {st['at_rate_limit']} 次")
    return lines


def run_protocol(record, input_fn=input, out=print, sleep=time.sleep, phases=PHASES):
    results = {}
    for i, (name, label, seconds, what) in enumerate(phases, 1):
        while True:
            out(f"\n[{i}/{len(phases)}] {label}（{seconds:.0f} 秒）\n   {what}")
            input_fn("   準備好就按 Enter 開始：")
            for c in (3, 2, 1):
                out(f"   {c}…")
                sleep(1.0)
            out("   錄製中…")
            samples = record(seconds)
            out(f"   錄完：{len(samples)} 筆")
            if input_fn("   保留這段嗎？ Enter=保留  r=重錄：").strip().lower() == "r":
                continue
            results[name] = samples
            break
    return results


def save(phases, data_dir=DATA_DIR, name=None):
    data_dir = Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    path = data_dir / (name or time.strftime("servo_response_%Y%m%d-%H%M%S.json"))
    if path.exists():
        raise FileExistsError(f"{path} 已經存在，不會覆蓋")
    path.write_text(json.dumps({"measured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                                "columns": ["t_s", "base_us", "shoulder_us", "elbow_us", "claw_us"],
                                "phases": {k: [list(s) for s in v] for k, v in phases.items()}}, indent=1) + "\n")
    return path


def main():
    print(__doc__.split("\n\n")[0])
    print("\n伺服電源要開著（伺服會照常跟著你動）。韌體要是伺服版的 phase3_control_loop，ST-Link 要接著。")
    test = record_swd(0.5)
    if not test:
        sys.exit("讀不到 TIM3 暫存器——ST-Link 有接嗎？板子有在跑嗎？（可用 check_hardware_ready.py --boot-check 確認）")
    print(f"ST-Link OK（0.5 秒讀到 {len(test)} 筆）。")
    phases = run_protocol(record_swd)
    path = save(phases)
    print("\n".join(summary_lines(phases)))
    print(f"\n已存到 {path}")


if __name__ == "__main__":
    main()

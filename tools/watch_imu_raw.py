"""Live raw view of both MPU6050s -- upper arm (0x68) and forearm (0x69) -- once a second, straight from the running
phase3_control_loop's UART. Nothing is flashed and nothing is judged into the numbers: so a person can look at the raw data
and decide for themselves whether the automatic health check (tools/sensor_health.py) is being too strict. The check's
verdict is printed last, separately, only for comparison.

Each second, per sensor: the last raw accel vector (g), |a| (about 1.0 when still), how many readings arrived, how many of
them were distinct (a live sensor is noisy: distinct ~ samples; a frozen one: 1), and the spread (max - min of each axis).
Then the firmware's own counters from its diag line: completed reads, nacks (no I2C answer), timeouts, re-wakes after a
failure, and PWR_MGMT_1 (1 = awake as configured, 64 = reset/asleep).

Usage:
    python3 tools/watch_imu_raw.py
    python3 tools/watch_imu_raw.py --port /dev/tty.usbserial-XXXXXXXX
    python3 tools/watch_imu_raw.py --no-verdict        # numbers only
"""
import argparse
import math
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sensor_health as sh  # noqa: E402

NAMES = {"upper_arm": ("上臂", "0x68", "shoulder"), "forearm": ("前臂", "0x69", "elbow")}


def _raw(text, fw):
    m = re.search(rf"{fw}_raw_ax=(\S+) {fw}_raw_ay=(\S+) {fw}_raw_az=(\S+)", text)
    return tuple(float(x) for x in m.groups()) if m else None


class Window:
    """One second's worth of lines."""

    def __init__(self):
        self.lines = 0
        self.samples = {name: [] for name in NAMES}
        self.hw = None

    def add_line(self, text):
        if "shoulder_completions=" in text and text.lstrip().startswith("diag"):
            self.hw = sh.parse_diag_line(text)
            return
        if "tick=" not in text:
            return
        self.lines += 1
        for name, (_label, _addr, fw) in NAMES.items():
            v = _raw(text, fw)
            if v is not None:
                self.samples[name].append(v)

    def summary(self, seconds):
        row = {"lines_per_s": self.lines / seconds if seconds > 0 else 0.0, "hw": self.hw}
        for name, vs in self.samples.items():
            if not vs:
                row[name] = {"last": None, "g": None, "samples": 0, "distinct": 0, "spread": None}
                continue
            last = vs[-1]
            row[name] = {"last": last, "g": math.sqrt(sum(c * c for c in last)), "samples": len(vs),
                         "distinct": len(set(vs)),
                         "spread": max(max(v[i] for v in vs) - min(v[i] for v in vs) for i in range(3))}
        return row


def format_row(row, verdict=None):
    parts = [f"{row['lines_per_s']:5.1f} 行/秒"]
    for name, (label, addr, _fw) in NAMES.items():
        r = row[name]
        if r["last"] is None:
            parts.append(f"{label}({addr}): 沒有資料")
            continue
        x, y, z = r["last"]
        parts.append(f"{label}({addr}): ({x:+.3f},{y:+.3f},{z:+.3f}) |a|={r['g']:.2f}g "
                     f"筆數={r['samples']} 不同值={r['distinct']} 最大變動={r['spread']:.3f}")
    text = "  |  ".join(parts)
    hw = row.get("hw")
    if hw:
        hw_parts = []
        for name, (label, _addr, _fw) in NAMES.items():
            c = hw[name]
            fields = " ".join(f"{k}={c[k]}" for k in ("nacks", "timeouts", "rewakes", "pwr_mgmt_1", "power_resets")
                              if c.get(k) is not None)
            hw_parts.append(f"{label} {fields}")
        text += "\n      韌體回報: " + "  |  ".join(hw_parts)
    if verdict:
        text += f"\n      (健康檢查的判斷,僅供對照: {verdict})"
    return text


def main():
    import serial
    from usb_serial_port import autodetect_port
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    parser.add_argument("--no-verdict", action="store_true", help="numbers only, no health-check verdict")
    args = parser.parse_args()
    port = args.port or autodetect_port()
    ser = serial.Serial(port, 115200, timeout=0.2)
    print(f"讀取 {port}(不燒錄、只讀)。Ctrl+C 結束。")
    monitor = sh.HealthMonitor()
    win, start = Window(), time.monotonic()
    try:
        while True:
            text = ser.readline().decode(errors="ignore")
            now = time.monotonic()
            win.add_line(text)
            up, fore = _raw(text, "shoulder"), _raw(text, "elbow")
            if up is not None or fore is not None:
                monitor.add_sample(now, up, fore)
            if win.hw is not None and text.lstrip().startswith("diag"):
                monitor.add_hardware(now, win.hw)
            if now - start >= 1.0:
                verdict = None
                if not args.no_verdict:
                    rep = monitor.report(now)
                    verdict = "正常" if rep.ok and not rep.warnings else \
                        "; ".join(f"{NAMES[p.sensor][0]}:{sh.KIND_TEXT[p.kind]}"
                                  for p in list(rep.problems) + list(rep.warnings))
                print(format_row(win.summary(now - start), verdict), flush=True)
                win, start = Window(), now
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()

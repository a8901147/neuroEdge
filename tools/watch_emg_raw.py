"""Live raw emg_min/emg_max trace, printed directly to the terminal so a
real relax/clench cycle can be watched (and timed) by the person doing it,
instead of a remote script guessing when to start capturing (found the hard
way, 2026-09-08: a blind timed capture driven by chat-message timing had no
real synchronization with the person's actual physical action, so the
resulting 16s trace was indistinguishable from flat noise even when a real
clench happened somewhere in that window).

Usage:
    python3 tools/watch_emg_raw.py
    python3 tools/watch_emg_raw.py --cp2102
    python3 tools/watch_emg_raw.py --port /dev/tty.usbserial-XXXXXXXX
"""
import argparse
import re
import sys
import time
from pathlib import Path

import serial

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))
from usb_serial_port import autodetect_port  # noqa: E402

LINE_RE = re.compile(r"tick=(\d+).*?emg_min=(?P<emin>\d+) emg_max=(?P<emax>\d+)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="default: auto-detect")
    parser.add_argument("--cp2102", action="store_true", help="use CP2102's fixed path instead of auto-detecting")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    port = args.port if args.port else autodetect_port(prefer_cp2102=args.cp2102)

    ser = serial.Serial(port, args.baud, timeout=0.5)
    print(f"Listening on {port} -- Ctrl+C to stop. Relax/clench whenever you like; "
          f"watch the bar move.")
    buf = b""
    try:
        while True:
            chunk = ser.read(1024)
            if not chunk:
                continue
            buf += chunk
            while b"\r\n" in buf:
                raw, buf = buf.split(b"\r\n", 1)
                line = raw.decode("utf-8", errors="ignore")
                m = LINE_RE.search(line)
                if not m:
                    if line.strip():
                        print(f"[FW] {line}")
                    continue
                emax = int(m.group("emax"))
                bar = "#" * min(80, emax // 25)
                print(f"emg_min={m.group('emin'):>4} emg_max={emax:>4} {bar}")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

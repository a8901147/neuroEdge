"""Live raw emg_min/emg_max trace, printed directly to the terminal so a
real relax/clench cycle can be watched (and timed) by the person doing it,
instead of a remote script guessing when to start capturing (found the hard
way, 2026-09-08: a blind timed capture driven by chat-message timing had no
real synchronization with the person's actual physical action, so the
resulting 16s trace was indistinguishable from flat noise even when a real
clench happened somewhere in that window).

Usage:
    python3 tools/watch_emg_raw.py
    python3 tools/watch_emg_raw.py --port /dev/tty.usbserial-0001
"""
import argparse
import re
import sys
import time

import serial

LINE_RE = re.compile(r"tick=(\d+).*?emg_min=(?P<emin>\d+) emg_max=(?P<emax>\d+)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/tty.usbserial-0001")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    print(f"Listening on {args.port} -- Ctrl+C to stop. Relax/clench whenever you like; "
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

#!/usr/bin/env python3
"""Live terminal view of firmware/src/timer_adc_1khz_main.c's UART output.

That firmware reports one line per ~1000 ADC samples (~once/second at the
hardware-timed 1kHz rate -- see PRD.md Phase 1.5 stage 3c/3d):

    samples=<count> min=<0-4095> max=<0-4095>

min/max are the smallest and largest raw 12-bit ADC readings seen during
that ~1-second window -- this is what actually shows a real muscle
contraction, since a single end-of-window snapshot can miss a contraction
that doesn't happen to land exactly on a report boundary.

This script just prints each line as it arrives, with a wall-clock
timestamp and the interval since the previous line (useful for eyeballing
whether the 1kHz claim is holding up in real time), plus a crude ASCII bar
for max so a growing/shrinking swing is visible without reading numbers.

Usage:
    python3 tools/watch_myoware_uart.py
    python3 tools/watch_myoware_uart.py --port /dev/tty.usbserial-0001 --baud 9600
"""
import argparse
import re
import time

import serial

LINE_RE = re.compile(r"samples=(\d+)\s+min=(\d+)\s+max=(\d+)")
ADC_FULL_SCALE = 4095


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/tty.usbserial-0001")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--bar-width", type=int, default=40, help="width of the ASCII max-value bar")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port} @ {args.baud} baud -- Ctrl+C to stop\n")

    prev_t = None
    buf = b""
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\r\n" in buf:
                raw, buf = buf.split(b"\r\n", 1)
                line = raw.decode(errors="replace")
                now = time.time()
                delta = "" if prev_t is None else f"(+{now - prev_t:5.3f}s)"
                prev_t = now

                match = LINE_RE.search(line)
                if match:
                    samples, lo, hi = (int(x) for x in match.groups())
                    swing = hi - lo
                    bar_len = int(hi / ADC_FULL_SCALE * args.bar_width)
                    bar = "#" * bar_len + "." * (args.bar_width - bar_len)
                    print(f"{delta:>10}  samples={samples:<8} min={lo:<5} max={hi:<5} "
                          f"swing={swing:<5} [{bar}]")
                else:
                    # Non-matching line (e.g. garbled on a fresh connection) -- show it raw.
                    print(f"{delta:>10}  {line!r}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()

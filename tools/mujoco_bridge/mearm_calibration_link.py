"""The MEArm calibration as the one-line UART message phase3_control_loop parses (2026-10-03, the user's choice "B":
send the calibration instead of compiling it in, so a re-calibration needs no re-flash). Format and parser:
include/edgeneuro/control/mearm_calibration_link.hpp --

    C<v0>,<v1>,...,<v19>,<checksum>\\n

20 values, each round(value * 1e6): HANG xyz, FORWARD xyz, LEFT xyz, RIGHT xyz, zero_elbow, has_reach (0 or 1e6),
base reach LEFT xyz, base reach RIGHT xyz (zeros when not measured); checksum = sum mod 1000000007. A calibration Path B
would reject is refused here, before anything is sent.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mearm_pathb as pb  # noqa: E402
import mearm_real as real  # noqa: E402

CHECKSUM_MODULUS = 1000000007
# The firmware reads ONE received byte per main-loop pass from a single-byte receive register, so bytes must arrive no
# faster than its slowest pass. Measured on the real board 2026-10-04: 8-byte chunks overran it on every message
# (uart_rx_overruns +2, message malformed); 1 byte per 1 ms lost 1 of 3; 1 byte per 2 ms applied 3 of 3, no overrun.
CHUNK_BYTES = 1
CHUNK_PAUSE_S = 0.002


def encode(saved):
    cal = pb.make_calibration(saved)                     # raises for a calibration Path B cannot use
    reach = real.saved_base_reach(cal, saved)            # raises for an unusable reach; None if not measured
    values = []
    for key in ("baseline_raw", "forward_raw", "left_twist_raw", "right_twist_raw"):
        values += list(saved[key])
    values.append(saved["zero_elbow"])
    values.append(1.0 if reach is not None else 0.0)
    for key in ("base_reach_left_raw", "base_reach_right_raw"):
        values += list(saved[key]) if reach is not None else [0.0, 0.0, 0.0]
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError("calibration has a non-finite value")
    ints = [int(round(float(v) * 1e6)) for v in values]
    return ("C" + "".join(f"{v}," for v in ints) + str(sum(ints) % CHECKSUM_MODULUS) + "\n").encode("ascii")


def send(ser, saved, sleep=time.sleep):
    msg = encode(saved)
    for i in range(0, len(msg), CHUNK_BYTES):
        ser.write(msg[i:i + CHUNK_BYTES])
        sleep(CHUNK_PAUSE_S)

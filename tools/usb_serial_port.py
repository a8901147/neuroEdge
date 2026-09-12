"""Shared USB-TTL adapter port resolution for every script in tools/ and
tools/mujoco_bridge/ that opens a live serial connection to the STM32.

Extracted 2026-09-12 from run_demo_live.py's own autodetect_port() after
CP2102 was replaced by FT232RL (2026-09-11, see SESSION_LOG.md's known
CP2102 firmware lockup bug) broke every script that hardcoded CP2102's
fixed path (/dev/tty.usbserial-0001) as its --port default -- FT232RL's
path isn't fixed the same way (macOS names it after the chip's own USB
serial string, e.g. /dev/tty.usbserial-A73C97JW, which differs per
physical unit), so hardcoding the new path in each script would just move
the same fragility somewhere else, once per copy. One shared, dependency-
free (no pyserial/mujoco import) module instead of duplicating this in
every script -- watch_emg_raw.py and watch_myoware_uart.py in particular
have no other reason to pull in anything heavier than the stdlib.

Usage from a script in tools/ or tools/mujoco_bridge/:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[N] / "tools"))
    from usb_serial_port import CP2102_PORT, autodetect_port
    # where N is however many parents/ steps reach the repo root from that
    # script's own location (1 for tools/, 2 for tools/mujoco_bridge/).
"""
import glob
import sys
from pathlib import Path

CP2102_PORT = "/dev/tty.usbserial-0001"


def autodetect_port(prefer_cp2102: bool = False) -> str:
    """Returns the port to use, resolved at call time (not import time) so
    a port that appears/disappears between script start and this call is
    handled correctly. Raises SystemExit with an actionable message rather
    than pyserial's raw FileNotFoundError if nothing is found."""
    if prefer_cp2102:
        if Path(CP2102_PORT).exists():
            return CP2102_PORT
        sys.exit(
            f"--cp2102 given but {CP2102_PORT} doesn't exist -- is the CP2102 adapter "
            "actually plugged in? (`ls /dev/cu.usbserial-*` to check what's connected)"
        )
    candidates = sorted(glob.glob("/dev/tty.usbserial-*"))
    if not candidates:
        sys.exit(
            "no /dev/tty.usbserial-* device found -- plug in the USB-TTL adapter "
            "(FT232RL or CP2102), or pass --port explicitly if it enumerates under a "
            "different name."
        )
    if len(candidates) > 1:
        print(f"[warn] multiple usbserial ports found ({candidates}), using {candidates[0]} "
              "-- pass --port explicitly to pick a different one.")
    return candidates[0]

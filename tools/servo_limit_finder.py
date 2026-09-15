"""Interactive companion to firmware/src/servo_limit_finder_main.c: lets a
person nudge a servo's PWM pulse width up/down by hand (over the same
USB-TTL UART every other tools/ script uses) to find its real safe range by
ear, instead of guessing a wider range and hoping. Flash
servo_limit_finder first (`cmake --build build --target
flash_servo_limit_finder`, then power-cycle the board -- see
SESSION_LOG.md's bootloader note), then run this.

Usage: type "+" then Enter to nudge up 25us, "-" then Enter to nudge down,
"q" then Enter to quit. Stop nudging the instant the servo sounds like it's
grinding against its own mechanical end-stop -- the last printed value
before that is this servo's real usable limit on that side.
"""
import sys
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from usb_serial_port import autodetect_port  # noqa: E402


def main():
    prefer_cp2102 = "--cp2102" in sys.argv
    port = autodetect_port(prefer_cp2102=prefer_cp2102)
    print(f"[servo_limit_finder] opening {port} @ 9600 baud")

    with serial.Serial(port, 9600, timeout=0.2) as ser:
        ser.reset_input_buffer()
        print("Type '+' then Enter to nudge up, '-' then Enter to nudge down, "
              "'q' then Enter to quit.")
        print("STOP as soon as you hear grinding/straining -- note the last "
              "printed pulse_us before that sound.")

        # Best-effort only: the firmware prints its boot banner exactly once,
        # right after power-up, whether or not anything is listening yet --
        # if this script is started even a moment after that power-cycle
        # (the normal sequence, since the board has to be unplugged/replugged
        # by hand first), the banner is already gone for good. So this reads
        # for at most a few timeout windows and moves on regardless, instead
        # of blocking forever waiting for a line that may never come.
        for _ in range(3):
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                print(f"  firmware: {line}")
        print("(if no 'firmware:' line showed up above, that's fine -- the "
              "boot banner only prints once and may have already been missed. "
              "Just start typing '+'/'-' below.)")

        while True:
            try:
                cmd = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if cmd == "q":
                break
            if cmd not in ("+", "-"):
                print("  (only '+', '-', or 'q' are recognized)")
                continue
            ser.write(cmd.encode("ascii"))
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                print(f"  firmware: {line}")


if __name__ == "__main__":
    main()

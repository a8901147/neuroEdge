#!/usr/bin/env python3
"""Interactive pose tool for the 4 MEArm servos -- for finding a safe rest pose by hand.

Needs `servo_limit_finder_4ch` flashed and the board power-cycled (it talks that
firmware's protocol: '1'-'4' select a channel, '+'/'-' move it 25us). This tool adds
what that firmware does not:
  * a target in microseconds (`set shoulder 1650`) instead of counting keypresses,
  * never leaves each servo's MEASURED range (SESSION_LOG 2026-09-22/23; kept equal to
    include/edgeneuro/control/mearm_servo_maps.hpp by a test),
  * always moves one 25us step at a time (never a jump), reading the real pulse back.

Commands (channels: 1=base 2=shoulder 3=elbow 4=claw, or the names):
  1 | 2 | 3 | 4         select a channel and show its pulse
  + [n]   - [n]         move the selected channel n steps of 25us (default 1)
  set <ch> <us>         walk one channel to a pulse width
  all <us>              base, shoulder and elbow to <us> (each clamped to its own range); the
                        claw always goes to its own rest (1300 = open), never to <us>
  rest                  the chosen rest pose: base/shoulder/elbow 1500, claw 1300
  show                  read all four back from the board
  q                     quit (prints the final pose as one line)

Usage:
    python3 tools/servo_pose_4ch.py
    python3 tools/servo_pose_4ch.py --port /dev/tty.usbserial-XXXX
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

NAMES = ("base", "shoulder", "elbow", "claw")
# Measured on the assembled MEArm, each with the other servos at centre.
RANGES = {1: (500, 2500), 2: (1200, 2100), 3: (500, 1850), 4: (1300, 1500)}   # claw: travel 1300..1600, limited to 1500 (user, 2026-10-03)
# The arm's chosen rest pose (2026-09-26); kept equal to mearm_servo_maps.hpp's k*RestUs by a test.
REST = {1: 1500, 2: 1500, 3: 1500, 4: 1300}
# claw and base first (they never interact with the linkage), then the elbow BEFORE the shoulder: a
# shoulder move with the elbow somewhere else could itself bind the linkage.
MOVE_ORDER = (4, 1, 3, 2)
STEP_US = 25            # what one '+'/'-' does in servo_limit_finder_4ch
STEP_DELAY_S = 0.04     # between steps: the walk is visible and the servo keeps up
BAUD = 115200


class BoardNotAnswering(RuntimeError):
    pass


def _channel(token):
    token = token.lower()
    if token in ("1", "2", "3", "4"):
        return int(token)
    if token in NAMES:
        return NAMES.index(token) + 1
    return None


def parse(line):
    """-> ('select', ch) | ('step', +-n) | ('set', ch, us) | ('all', us) | ('show',) | ('quit',) | ('error', why)"""
    parts = line.strip().lower().split()
    if not parts:
        return ("error", "empty command")
    head = parts[0]
    if head in ("q", "quit", "exit"):
        return ("quit",)
    if head == "show":
        return ("show",)
    if head == "rest" and len(parts) == 1:
        return ("rest",)
    if head in ("1", "2", "3", "4") and len(parts) == 1:
        return ("select", int(head))
    if head[0] in "+-":
        rest = parts[1] if len(parts) > 1 else head[1:]
        try:
            n = int(rest) if rest else 1
        except ValueError:
            return ("error", "the step count must be an integer")
        if len(parts) > 2 or n < 0:
            return ("error", "usage: + [steps]")
        return ("step", n if head[0] == "+" else -n)
    if head == "set" and len(parts) == 3:
        ch = _channel(parts[1])
        if ch is None:
            return ("error", "the channel must be 1-4 or base/shoulder/elbow/claw")
        try:
            return ("set", ch, int(parts[2]))
        except ValueError:
            return ("error", "the pulse width must be an integer (µs)")
    if head == "all" and len(parts) == 2:
        try:
            return ("all", int(parts[1]))
        except ValueError:
            return ("error", "the pulse width must be an integer (µs)")
    return ("error", "unrecognized command")


def clamp_target(ch, target_us):
    """(clamped_us, was_clamped) -- kept inside the channel's measured range."""
    lo, hi = RANGES[ch]
    clamped = min(max(target_us, lo), hi)
    return clamped, clamped != target_us


def plan_steps(current_us, target_us):
    """(direction, number_of_25us_steps) to get from current to the nearest grid point of target."""
    delta = target_us - current_us
    n = int(abs(delta) / STEP_US + 0.5)
    return (+1 if delta >= 0 else -1), n


def read_state(board, ch, wait=0.3):
    """Select channel ch (the firmware prints its state on select) and return its pulse in us."""
    import re
    board.reset_input_buffer()
    board.write(str(ch).encode())
    deadline = time.monotonic() + max(wait, 0.0) + 1.0
    text = ""
    while time.monotonic() < deadline:
        time.sleep(0.02)
        if board.in_waiting:
            text += board.read(board.in_waiting).decode(errors="replace")
            m = re.search(r"pulse_us=(\d+)", text)
            if m:
                return int(m.group(1))
    raise BoardNotAnswering(
        "the board does not respond. Is servo_limit_finder_4ch the firmware on it, and did the app start after flashing "
        "(check with check_hardware_ready.py --boot-check)?")


def _walk(board, ch, target_us):
    """Walk channel ch to target_us one step at a time; returns the pulse read back."""
    target, clamped = clamp_target(ch, target_us)
    if clamped:
        lo, hi = RANGES[ch]
        print(f"  {NAMES[ch - 1]} is limited to {lo}-{hi} µs (its measured range); using {target}")
    current = read_state(board, ch)
    direction, n = plan_steps(current, target)
    key = b"+" if direction > 0 else b"-"
    for _ in range(n):
        board.write(key)
        time.sleep(STEP_DELAY_S)
    return read_state(board, ch)


def session(board, prompt="pose> "):
    pose = {ch: None for ch in RANGES}
    selected = 1
    while True:
        try:
            line = input(prompt)
        except EOFError:
            line = "q"
        cmd = parse(line)
        try:
            if cmd[0] == "quit":
                break
            elif cmd[0] == "error":
                print(f"  {cmd[1]}. Type show / rest / set shoulder 1500 / all 1500 / + 4 / q")
            elif cmd[0] == "select":
                selected = cmd[1]
                pose[selected] = read_state(board, selected)
                print(f"  channel={NAMES[selected - 1]} pulse_us={pose[selected]}")
            elif cmd[0] == "step":
                n = cmd[1]
                current = read_state(board, selected)
                target = current + n * STEP_US
                pose[selected] = _walk(board, selected, target)
                print(f"  channel={NAMES[selected - 1]} pulse_us={pose[selected]}")
            elif cmd[0] == "set":
                selected = cmd[1]
                pose[selected] = _walk(board, selected, cmd[2])
                print(f"  channel={NAMES[selected - 1]} pulse_us={pose[selected]}")
            elif cmd[0] in ("all", "rest"):
                for ch in MOVE_ORDER:
                    selected = ch
                    target = REST[ch] if (cmd[0] == "rest" or ch == 4) else cmd[1]     # the claw never follows <us>
                    pose[ch] = _walk(board, ch, target)
                print("  " + "  ".join(f"{NAMES[ch - 1]}={pose[ch]}" for ch in RANGES))
            elif cmd[0] == "show":
                for ch in RANGES:
                    pose[ch] = read_state(board, ch)
                    print(f"  {NAMES[ch - 1]:9s}{pose[ch]}")
                selected = 4
        except BoardNotAnswering as exc:
            print(f"  {exc}")
    for ch in RANGES:                                   # final read-back, not the last thing we believed
        try:
            pose[ch] = read_state(board, ch)
        except BoardNotAnswering:
            pass
    line = " ".join(f"{NAMES[ch - 1]}={pose[ch]}" for ch in RANGES)
    print(f"\nFinal pose: {line}")
    return pose


def main():
    import serial
    from usb_serial_port import CP2102_PORT, autodetect_port
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None)
    parser.add_argument("--cp2102", action="store_true", help=f"use the CP2102 adapter at {CP2102_PORT}")
    args = parser.parse_args()
    port = args.port or autodetect_port(prefer_cp2102=args.cp2102)
    board = serial.Serial(port, BAUD, timeout=0.5)
    print(f"Connected to {port}. Type show for the four current pulse widths, rest for the rest pose, q to quit.")
    session(board)


if __name__ == "__main__":
    main()

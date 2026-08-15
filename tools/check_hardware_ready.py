#!/usr/bin/env python3
"""Pre-flight hardware check for the Phase 1.5 STM32 spike.

Run this BEFORE flashing/reading UART, instead of discovering mid-task
that the USB-TTL adapter silently dropped -- this exact failure mode
recurred repeatedly during development (see PRD.md Phase 1.5): the
/dev/tty.usbserial-* device node can exist while the port is still
unusable (opening it raises a termios EINVAL), so a bare `ls` or even
`system_profiler` alone is NOT enough to confirm the adapter is really
ready -- this script actually opens and configures the port, which is
the same operation flashing/reading will need to do.

Usage:
    python3 tools/check_hardware_ready.py
"""
import glob
import subprocess
import sys


def check_usb_device(vendor_substr: str) -> bool:
    try:
        out = subprocess.run(
            ["system_profiler", "SPUSBDataType"], capture_output=True, text=True, timeout=15
        ).stdout
    except Exception:
        return False
    return vendor_substr.lower() in out.lower()


def check_serial_port():
    """Returns (ok, port_or_None, error_or_None)."""
    candidates = glob.glob("/dev/tty.usbserial-*")
    if not candidates:
        return False, None, "no /dev/tty.usbserial-* device node found"
    port = candidates[0]
    try:
        import serial
    except ImportError:
        return False, port, "pyserial not installed (pip3 install pyserial)"
    try:
        ser = serial.Serial(port, 9600, timeout=1)
        ser.close()
        return True, port, None
    except Exception as e:
        return False, port, str(e)


def main() -> None:
    all_ok = True

    stlink_ok = check_usb_device("STMicroelectronics")
    print(f"[{'OK  ' if stlink_ok else 'FAIL'}] ST-Link enumerated over USB")
    all_ok = all_ok and stlink_ok

    ttl_ok = check_usb_device("Silicon Labs")
    print(f"[{'OK  ' if ttl_ok else 'FAIL'}] USB-TTL (CP2102/Silicon Labs) enumerated over USB")
    all_ok = all_ok and ttl_ok

    port_ok, port, err = check_serial_port()
    label = port or "/dev/tty.usbserial-*"
    suffix = f" -- {err}" if err else ""
    print(f"[{'OK  ' if port_ok else 'FAIL'}] {label} opens and configures cleanly{suffix}")
    all_ok = all_ok and port_ok

    if not all_ok:
        print(
            "\nNot ready. If the USB-TTL checks failed: fully unplug the adapter, wait "
            "~10 real seconds, then plug it back in ONCE -- rapid unplug/replug cycles "
            "have repeatedly left the port in this exact stuck state during development, "
            "while one deliberate slow cycle has always fixed it. Re-run this script after."
        )
        sys.exit(1)

    print("\nAll checks passed -- safe to flash/read.")
    sys.exit(0)


if __name__ == "__main__":
    main()

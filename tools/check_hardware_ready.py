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
    python3 tools/check_hardware_ready.py --i2c-scan

--i2c-scan additionally flashes firmware/src/i2c_bus_scan_main.c (a
permanent diagnostic target, not one of the pipeline stages) and reads
its results back over SWD via OpenOCD. This automates the exact manual
process used to debug Phase 1.5 Stage 4b: MPU6050/GY-521 stopped
responding after repeated breadboard handling, and it took a long,
mostly-manual SWD session (BUSY-bit checks, address/pin/peripheral
swaps, a full 128-address scan cross-tested against a known-good LCD1602
module) to prove the STM32 side was healthy and the sensor module itself
was at fault -- see PRD.md Phase 1.5 Stage 4b for the full writeup. This
flag exists so that process doesn't have to be reinvented by hand next
time an I2C device goes quiet: it flashes the scanner, waits for it to
finish, and reports which addresses (if any) ACKed, plus whether the bus
was already stuck BUSY before the scan even started.
"""
import argparse
import glob
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
FIRMWARE_DIR = REPO_ROOT / "firmware"
BUILD_DIR = FIRMWARE_DIR / "build"
OPENOCD_CFG = FIRMWARE_DIR / "openocd.cfg"
SCAN_TARGET = "i2c_bus_scan"


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


def run_basic_checks() -> tuple:
    """Returns (all_ok, stlink_ok) -- stlink_ok is exposed separately since
    the I2C scan only needs SWD (ST-Link), not the USB-TTL/UART checks."""
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
    return all_ok, stlink_ok


def _nm_addresses(binary: Path) -> dict:
    out = subprocess.run(
        ["arm-none-eabi-nm", str(binary)], capture_output=True, text=True, timeout=15
    ).stdout
    addrs = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            addr, _type, name = parts
            addrs[name] = int(addr, 16)
    return addrs


def _mdw_read(addr: int, count: int = 1) -> list:
    result = subprocess.run(
        [
            "openocd", "-f", str(OPENOCD_CFG),
            "-c", "init", "-c", "halt",
            "-c", f"mdw 0x{addr:08x} {count}",
            "-c", "resume", "-c", "shutdown",
        ],
        capture_output=True, text=True, timeout=30,
    )
    text = result.stdout + result.stderr
    m = re.search(rf"0x{addr:08x}:((?:\s+[0-9a-f]{{8}})+)", text)
    if not m:
        raise RuntimeError(f"could not parse mdw output for 0x{addr:08x}:\n{text}")
    return [int(w, 16) for w in m.group(1).split()]


def run_i2c_scan() -> bool:
    print("\n--- I2C1 bus scan (flashes firmware/src/i2c_bus_scan_main.c) ---")
    if not BUILD_DIR.exists():
        print(f"[FAIL] {BUILD_DIR} does not exist -- run cmake configure first")
        return False

    build = subprocess.run(
        ["cmake", "--build", str(BUILD_DIR), "--target", SCAN_TARGET],
        capture_output=True, text=True, timeout=120,
    )
    if build.returncode != 0:
        print("[FAIL] build failed:\n" + build.stdout[-2000:] + build.stderr[-2000:])
        return False

    binary = BUILD_DIR / SCAN_TARGET
    addrs = _nm_addresses(binary)
    required = ["g_scan_bitmap", "g_scan_done", "g_bus_busy_before_scan"]
    missing = [n for n in required if n not in addrs]
    if missing:
        print(f"[FAIL] symbols missing from binary (rebuild stale?): {missing}")
        return False

    flash = subprocess.run(
        ["cmake", "--build", str(BUILD_DIR), "--target", f"flash_{SCAN_TARGET}"],
        capture_output=True, text=True, timeout=60,
    )
    flash_out = flash.stdout + flash.stderr
    # Not gating on returncode: OpenOCD's `shutdown` command commonly exits
    # non-zero even after a clean, verified flash -- the text markers are
    # the reliable signal.
    if "** Programming Finished **" not in flash_out or "** Verified OK **" not in flash_out:
        print("[FAIL] flash failed:\n" + flash_out[-2000:])
        return False
    print("[OK  ] flashed i2c_bus_scan")

    import time
    time.sleep(1.5)  # 126-address scan completes well within this

    busy_before = _mdw_read(addrs["g_bus_busy_before_scan"])[0]
    scan_done = _mdw_read(addrs["g_scan_done"])[0]
    bitmap = _mdw_read(addrs["g_scan_bitmap"], 4)

    # g_bus_busy_before_scan holds I2C1->SR2 & I2C_SR2_BUSY -- the raw
    # masked bit value (0x2 for the BUSY bit, not a normalized 0/1), so
    # "busy" is "nonzero", not "== 1". (Found this the hard way: it
    # mis-reported a real stuck-BUSY case as an "unexpected" value instead
    # of the proper FAIL below.)
    if busy_before == 0:
        print("[OK  ] I2C1 bus was not BUSY before the scan")
    else:
        print(
            f"[FAIL] I2C1 bus was already BUSY (stuck) before the scan even started (raw=0x{busy_before:08x}) -- "
            "a device is holding SDA or SCL low, or the peripheral's internal state is "
            "wedged. Check module seating, then re-run (SWRST in i2c1_init() should "
            "clear a wedged peripheral state, but not a genuinely shorted/held line)."
        )

    if scan_done != 1:
        print(f"[FAIL] scan did not complete (g_scan_done=0x{scan_done:08x}) -- target may be halted/crashed")
        return False

    found = []
    for word_idx, word in enumerate(bitmap):
        for bit in range(32):
            if word & (1 << bit):
                found.append(word_idx * 32 + bit)

    if found:
        addr_list = ", ".join(f"0x{a:02x}" for a in found)
        print(f"[OK  ] scan complete -- ACK from: {addr_list}")
    else:
        print(
            "[FAIL] scan complete -- no address ACKed. If a device is definitely "
            "connected: re-check power (measure VCC-GND directly, don't trust an "
            "onboard LED alone), then SCL/SDA continuity end-to-end. If those are "
            "all fine, suspect the connected module itself -- cross-test with a "
            "known-good I2C device (e.g. a PCF8574 LCD backpack, address 0x27) "
            "before assuming the STM32 side is at fault."
        )
    return bool(found) and busy_before == 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--i2c-scan", action="store_true",
        help="also flash the I2C1 bus scanner and report which addresses ACK",
    )
    args = parser.parse_args()

    basic_ok, stlink_ok = run_basic_checks()

    i2c_ok = True
    if args.i2c_scan:
        if not stlink_ok:
            print("\n[SKIP] I2C scan needs a working ST-Link -- fix the above first")
            i2c_ok = False
        else:
            i2c_ok = run_i2c_scan()

    if not (basic_ok and i2c_ok):
        sys.exit(1)

    print("\nAll checks passed -- safe to flash/read.")
    sys.exit(0)


if __name__ == "__main__":
    main()

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
    python3 tools/check_hardware_ready.py --live-check

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

--live-check flashes the REAL Stage 6 firmware (phase3_control_loop, not
the scanner) and checks the actual thing the MuJoCo bridge depends on:
both MPU6050 wake-up writes succeeding (g_wake_result_shoulder/elbow via
`mdw`), and a few live seconds of real UART output actually containing
non-default, changing shoulder_pitch/shoulder_roll/elbow values -- not
just "the address ACKed" (--i2c-scan) or "the port opens" (the basic
checks), which both passed multiple times during Stage 6 bring-up while
the arm was still completely frozen in the MuJoCo viewer, either because
the I2C bus was stuck in a way that only shows up once the real control
loop is running, or because of an unrelated bug (a zero-offset/clamp
mismatch between the real sensors' absolute mounting angle and the
simulated joint ranges) that no I2C-level check could ever catch. This
flag automates the exact multi-step manual sequence (reflash, `mdw` the
wake results, capture and parse raw serial lines) that kept getting
re-typed by hand during that debugging session.
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
LIVE_TARGET = "phase3_control_loop"

# firmware/CMakeLists.txt's APP_FLASH_ADDRESS -- every real target (scanner
# or phase3) links here. The first 16KB below it (0x08000000-0x08003fff) is
# the WeAct HID bootloader's own flash, and 0x1fff0000+ is the STM32's
# factory ROM bootloader -- three completely different "why is there no
# UART output" causes that all look identical from a UART capture alone,
# see check_boot_reached_app()'s own comment.
APP_FLASH_ADDRESS = 0x08004000
ROM_BOOTLOADER_BASE = 0x1FFF0000

# Same pattern tools/mujoco_bridge/run_demo_live.py parses -- kept in sync
# by hand since one lives in Python tooling and the other's authoritative
# copy is the firmware's own usart2 print statements; a format drift here
# would show up as "0 valid lines" below, not a silent false-pass.
LIVE_LINE_RE = re.compile(
    r"tick=(?P<tick>\d+) grip=(?P<grip>[-\d.eE+]+) gripping=(?P<gripping>\d) "
    r"shoulder_pitch=(?P<shoulder_pitch>[-\d.eE+]+) shoulder_roll=(?P<shoulder_roll>[-\d.eE+]+) "
    r"elbow=(?P<elbow>[-\d.eE+]+)"
)

# `elbow` above is NOT a raw per-IMU reading -- it's shoulder_pitch minus
# elbow_pitch, clamped to >=0 (see phase3_control_loop_main.cpp's sign-
# verification comment), so a real, live elbow IMU can still make `elbow`
# read frozen at exactly 0.0 for an entire capture window whenever the arm
# happens to sit in the pose where that difference is negative -- confirmed
# 2026-08-23, twice, on hardware that diag_shoulder/elbow_completions proved
# was completely healthy both times. This diag line's completions/nacks/
# timeouts are a real per-IMU I2C-level signal, not a derived/clamped value,
# so they're what --live-check actually uses to judge shoulder/elbow health
# below; the `elbow` field itself is only still checked for gross staleness
# (frozen means the port's fully wedged, not that this specific IMU is bad).
DIAG_LINE_RE = re.compile(
    r"diag shoulder_completions=(?P<shoulder_completions>\d+) "
    r"elbow_completions=(?P<elbow_completions>\d+) "
    r"shoulder_nacks=(?P<shoulder_nacks>\d+) shoulder_timeouts=(?P<shoulder_timeouts>\d+) "
    r"elbow_nacks=(?P<elbow_nacks>\d+) elbow_timeouts=(?P<elbow_timeouts>\d+)"
)


def check_usb_device(*vendor_substrs: str) -> bool:
    try:
        out = subprocess.run(
            ["system_profiler", "SPUSBDataType"], capture_output=True, text=True, timeout=15
        ).stdout
    except Exception:
        return False
    out = out.lower()
    return any(v.lower() in out for v in vendor_substrs)


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
        # 115200 matches phase3_control_loop_main.cpp's Stage 6 USART2 baud
        # (bumped from 9600 to sustain 100Hz of the dual-IMU output line --
        # see that file's usart2_init() comment). Opening at the "wrong"
        # baud wouldn't actually fail this check either way (baud is a
        # local config, not negotiated with the device), but keep it
        # matching the firmware currently being brought up.
        ser = serial.Serial(port, 115200, timeout=1)
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

    # FTDI (FT232RL) replaced the CP2102 2026-09-11 -- see
    # project_bootloader_requires_power_cycle memory / PRD.md's 2026-09-10
    # handoff for why (CP2102's known firmware lockup bug). Both substrings
    # kept so this still works if a CP2102 is ever plugged in again.
    ttl_ok = check_usb_device("Silicon Labs", "FTDI")
    print(f"[{'OK  ' if ttl_ok else 'FAIL'}] USB-TTL (FT232RL/FTDI or CP2102/Silicon Labs) enumerated over USB")
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


def _read_pc() -> int:
    result = subprocess.run(
        [
            "openocd", "-f", str(OPENOCD_CFG),
            "-c", "init", "-c", "halt", "-c", "reg pc", "-c", "resume", "-c", "shutdown",
        ],
        capture_output=True, text=True, timeout=30,
    )
    text = result.stdout + result.stderr
    m = re.search(r"pc:\s*0x([0-9a-fA-F]+)", text)
    if not m:
        raise RuntimeError(f"could not parse `reg pc` output:\n{text}")
    return int(m.group(1), 16)


def check_boot_reached_app(poll_seconds: float = 25.0) -> bool:
    """Ground-truth, application-independent check: does execution actually
    reach the flashed app at all? Found 2026-09-10 the hard way -- both
    run_i2c_scan()'s "scan did not complete" and run_live_check()'s "0
    valid lines parsed" look identical whether the real cause is (a) a UART/
    CP2102 problem, (b) an I2C device (e.g. MPU6050) not answering and the
    app's own fail-safe hanging in blink_code() before ever reaching the
    UART prints, or (c) execution never reaching the app's main() at all --
    three unrelated failure classes that all present as "no output". This
    function isolates (c) via SWD alone, with no dependency on anything the
    app itself does, so it can't be masked by an earlier app-level failure.

    Also empirically confirmed 2026-09-10, repeatedly: right after any SWD
    flash (`flash_<target>` via `program ... reset exit` -- openocd's own
    `reset` is a warm/pin reset), PC reliably reads stuck inside the
    bootloader no matter how long you poll, and only a genuine physical
    unplug-wait-replug of the board's power makes it jump to the app. Why
    this is true is NOT confirmed -- two guesses (a "board's native USB
    plugged into a host" theory, and later a "bootloader deliberately
    distinguishes POR from pin reset" theory) were both written down here
    at different points and neither held up to checking (see
    project_bootloader_requires_power_cycle memory for the walk-back and
    what's actually known: WeAct's bootloader binary is closed-source, so
    the real mechanism is unverified). Treat this purely as an operating
    rule, not an explained one: right after a flash, this check is EXPECTED
    to read "stuck in bootloader" until a human physically power-cycles the
    board -- so this polls for up to poll_seconds (prompting once) instead
    of a single immediate read, giving that a real window to happen.
    """
    print("\n--- Boot sanity check (does execution actually reach the app?) ---")
    print(
        f"    If this was just flashed: fully unplug the board's power cable, wait a "
        f"couple seconds, then plug it back in. Polling for up to {poll_seconds:.0f}s..."
    )
    import time
    deadline = time.time() + poll_seconds
    last_pc = None
    while True:
        pc = _read_pc()
        last_pc = pc
        if pc >= APP_FLASH_ADDRESS:
            print(f"[OK  ] PC=0x{pc:08x} is inside the app (>= 0x{APP_FLASH_ADDRESS:08x}) -- execution reached main()")
            return True
        if time.time() >= deadline:
            break
        time.sleep(1.5)

    pc = last_pc
    if ROM_BOOTLOADER_BASE <= pc < ROM_BOOTLOADER_BASE + 0x8000:
        print(
            f"[FAIL] PC=0x{pc:08x} is in the STM32's factory ROM bootloader (0x{ROM_BOOTLOADER_BASE:08x}+) -- "
            "BOOT0 was read high at the last reset. Check the BOOT0 pin/jumper is low, then "
            "power-cycle again."
        )
        return False
    print(
        f"[FAIL] PC=0x{pc:08x} is still inside the WeAct HID bootloader (0x08000000-0x08003fff) "
        f"after {poll_seconds:.0f}s -- a power-cycle either didn't happen or didn't take. Try a "
        "slower, more deliberate unplug/wait/replug cycle."
    )
    return False


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

    if not check_boot_reached_app():
        return False

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


def run_live_check(port: str) -> bool:
    print(f"\n--- Live Stage 6 check (flashes firmware/src/{LIVE_TARGET}_main.cpp) ---")
    if not BUILD_DIR.exists():
        print(f"[FAIL] {BUILD_DIR} does not exist -- run cmake configure first")
        return False

    build = subprocess.run(
        ["cmake", "--build", str(BUILD_DIR), "--target", LIVE_TARGET],
        capture_output=True, text=True, timeout=120,
    )
    if build.returncode != 0:
        print("[FAIL] build failed:\n" + build.stdout[-2000:] + build.stderr[-2000:])
        return False

    binary = BUILD_DIR / LIVE_TARGET
    addrs = _nm_addresses(binary)
    required = ["g_wake_result_shoulder", "g_wake_result_elbow"]
    missing = [n for n in required if n not in addrs]
    if missing:
        print(f"[FAIL] symbols missing from binary (rebuild stale?): {missing}")
        return False

    flash = subprocess.run(
        ["cmake", "--build", str(BUILD_DIR), "--target", f"flash_{LIVE_TARGET}"],
        capture_output=True, text=True, timeout=60,
    )
    flash_out = flash.stdout + flash.stderr
    if "** Programming Finished **" not in flash_out or "** Verified OK **" not in flash_out:
        print("[FAIL] flash failed:\n" + flash_out[-2000:])
        return False
    print(f"[OK  ] flashed {LIVE_TARGET}")

    if not check_boot_reached_app():
        return False

    try:
        import serial
    except ImportError:
        print("[FAIL] pyserial not installed (pip3 install pyserial) -- can't capture live data")
        return False
    ser = serial.Serial(port, 115200, timeout=1)

    import time
    time.sleep(1.0)  # let both wake-up writes (each blocking, at boot) complete

    wake_shoulder = _mdw_read(addrs["g_wake_result_shoulder"])[0]
    wake_elbow = _mdw_read(addrs["g_wake_result_elbow"])[0]
    # Both are `int`, so a nonzero mdw word IS the failure code already
    # (no BUSY-style raw-vs-normalized bit-mask gotcha here, unlike
    # g_bus_busy_before_scan above). 0xffffffff (still the linker's -1
    # initializer) means this specific write was never even attempted --
    # e.g. the shoulder write NACKed and, if required, the app is now
    # hanging in blink_code() before ever reaching the elbow write. Each
    # code is mpu6050_write_reg_blocking()'s own return value: 1/2/3/4 =
    # timed out waiting for SB/ADDR-or-NACK/TXE-or-BTF/BTF respectively.
    wake_ok = wake_shoulder == 0 and wake_elbow == 0
    if wake_ok:
        print("[OK  ] both MPU6050 wake-up writes succeeded (g_wake_result_shoulder/elbow == 0)")
    else:
        def _describe(name, code):
            if code == 0xFFFFFFFF or code == -1:
                return f"{name}: never attempted (an earlier required sensor likely hung the boot in blink_code())"
            if code == 0:
                return f"{name}: OK"
            return f"{name}: FAILED, code={code} (1/2=NACK/timeout on I2C address phase, 3/4=timeout writing register/value)"
        print(
            "[FAIL] wake-up write failed -- "
            f"{_describe('shoulder', wake_shoulder)}; {_describe('elbow', wake_elbow)}. "
            "This is a per-device I2C result, independent of UART -- check that specific "
            "sensor's VCC/GND/SDA/SCL wiring. Use --i2c-scan to test both devices' presence "
            "without depending on either one's wake write succeeding first."
        )

    samples = []
    diag_samples = []
    try:
        print(f"capturing ~3s of live UART data on {port} @ 115200 baud...")
        buf = b""
        deadline = time.time() + 3.0
        while time.time() < deadline:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\r\n" in buf:
                raw, buf = buf.split(b"\r\n", 1)
                line = raw.decode("utf-8", errors="ignore")
                match = LIVE_LINE_RE.search(line)
                if match:
                    samples.append({k: float(v) if k != "tick" and k != "gripping" else int(v)
                                     for k, v in match.groupdict().items()})
                    continue
                diag_match = DIAG_LINE_RE.search(line)
                if diag_match:
                    diag_samples.append({k: int(v) for k, v in diag_match.groupdict().items()})
        ser.close()
    except Exception as e:
        print(f"[FAIL] could not read {port}: {e}")
        return False

    if not samples:
        print(
            "[FAIL] 0 valid lines parsed in 3s -- either nothing is being transmitted "
            "(firmware stuck, e.g. retrying a wedged I2C bus after boot) or the line "
            "format drifted from LIVE_LINE_RE above. Try `python3 -m serial.tools.miniterm "
            f"{port} 115200` to see the raw output directly."
        )
        return False
    print(f"[OK  ] {len(samples)} lines parsed")

    ticks = [s["tick"] for s in samples]
    tick_alive = ticks[-1] > ticks[0] if len(ticks) > 1 else False
    print(f"[{'OK  ' if tick_alive else 'FAIL'}] tick counter advancing "
          f"({ticks[0]} -> {ticks[-1]})" if ticks else "[FAIL] no tick values")

    # shoulder_pitch/shoulder_roll are raw ComplementaryFilter output (not
    # derived/clamped like `elbow`, see DIAG_LINE_RE's comment above), so a
    # stuck/never-initialized filter reading exactly 0.0 with zero variance
    # forever is still a meaningful, real signal here -- that was the exact
    # symptom that cost the most debugging time during Stage 6 bring-up
    # (data WAS flowing at 100Hz, tick WAS advancing, but shoulder_pitch/
    # roll were frozen at precisely 0.000000, meaning the I2C reads behind
    # them were silently never completing even once since boot).
    data_ok = True
    for field in ("shoulder_pitch", "shoulder_roll"):
        values = [s[field] for s in samples]
        spread = max(values) - min(values)
        stuck = spread < 1e-6
        if stuck:
            data_ok = False
        print(f"[{'FAIL' if stuck else 'OK  '}] {field}: "
              f"{'frozen at ' + str(values[0]) if stuck else f'varying (spread={spread:.4f})'}")

    # Per-IMU health via the diag line's completions/nacks/timeouts, not
    # `elbow`'s spread -- `elbow` is shoulder_pitch minus elbow_pitch,
    # clamped to >=0, so a perfectly healthy elbow IMU can still make it
    # read frozen at 0.0 for an entire 3s window whenever the arm happens to
    # sit in the pose where that difference goes negative. Confirmed
    # 2026-08-23, twice, against hardware these completions counters proved
    # was completely healthy both times -- see run_demo_live.py's
    # imu_liveness() for the same reasoning applied there.
    if not diag_samples:
        print(
            "[FAIL] no diag line (shoulder/elbow_completions) seen in 3s -- the firmware "
            "emits one about once a second, so this capture window should have caught one; "
            "re-run, or check for a LIVE_LINE_RE/DIAG_LINE_RE format drift."
        )
        data_ok = False
    else:
        last_diag = diag_samples[-1]
        for name, addr, completions_key, nacks_key, timeouts_key in [
            ("shoulder IMU (0x68)", "shoulder", "shoulder_completions", "shoulder_nacks", "shoulder_timeouts"),
            ("elbow IMU (0x69)", "elbow", "elbow_completions", "elbow_nacks", "elbow_timeouts"),
        ]:
            completions = last_diag[completions_key]
            imu_ok = completions > 0
            if not imu_ok:
                data_ok = False
                nacks, timeouts = last_diag[nacks_key], last_diag[timeouts_key]
                cause = (f"timing out (bus wedged, timeouts={timeouts})" if timeouts > 0
                         else f"NACKing (device not answering, nacks={nacks})" if nacks > 0
                         else "not completing (cause unclear from this window)")
                print(f"[FAIL] {name}: 0 completions in the last ~1s window -- {cause}")
            else:
                print(f"[OK  ] {name}: {completions} completions in the last ~1s window")

    if not data_ok:
        print(
            "\nA frozen shoulder_pitch/roll field, or an IMU showing 0 completions above, "
            "means that IMU's reads have never completed successfully since boot, even "
            "though the port is open and other fields are updating -- re-check that "
            "specific sensor's wiring (see PRD.md Stage 6), not the STM32 side, which the "
            "above wake/tick checks already confirmed is healthy."
        )

    return wake_ok and tick_alive and data_ok


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--i2c-scan", action="store_true",
        help="also flash the I2C1 bus scanner and report which addresses ACK",
    )
    parser.add_argument(
        "--live-check", action="store_true",
        help="flash the real Stage 6 firmware and verify live shoulder/roll/elbow data actually changes",
    )
    parser.add_argument(
        "--boot-check", action="store_true",
        help="standalone: poll (no reflash) for execution to reach whatever is currently "
             "flashed -- run this right after you've manually power-cycled the board",
    )
    args = parser.parse_args()

    if args.boot_check:
        sys.exit(0 if check_boot_reached_app() else 1)

    basic_ok, stlink_ok = run_basic_checks()

    i2c_ok = True
    if args.i2c_scan:
        if not stlink_ok:
            print("\n[SKIP] I2C scan needs a working ST-Link -- fix the above first")
            i2c_ok = False
        else:
            i2c_ok = run_i2c_scan()

    live_ok = True
    if args.live_check:
        _, port, _ = check_serial_port()
        if not stlink_ok or not port:
            print("\n[SKIP] live check needs both a working ST-Link and USB-TTL port -- fix the above first")
            live_ok = False
        else:
            live_ok = run_live_check(port)

    if not (basic_ok and i2c_ok and live_ok):
        sys.exit(1)

    print("\nAll checks passed -- safe to flash/read.")
    sys.exit(0)


if __name__ == "__main__":
    main()

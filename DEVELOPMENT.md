# Development notes

How to build, flash and test this repository, and the rules its code and documents follow.
Design and requirements are in [PRD.md](PRD.md); dated findings are in [SESSION_LOG.md](SESSION_LOG.md).

## Build and test

```sh
# Host C++ tests (Debug + malloc_count==0 heap guard)
cmake --preset debug-heapguard && cmake --build --preset debug-heapguard -j
ctest --preset debug-heapguard --output-on-failure

# Host C++ tests (ASan + UBSan)
cmake --preset sanitize-asan-ubsan && cmake --build --preset sanitize-asan-ubsan -j
ctest --preset sanitize-asan-ubsan --output-on-failure

# Firmware (arm-none-eabi cross-compile)
cd firmware && cmake -S . -B build && cmake --build build --target <name>
cmake --build build --target flash_<name>   # needs ST-Link attached

# Python tests (tools/, tools/mujoco_bridge/): .github/workflows/ci.yml lists what CI runs.
```

## Starting the app after an SWD flash

After an SWD flash the WeAct bootloader often does not jump to the app (unplug/replug and `reset run` are both
unreliable). Start the app by jumping straight to its Reset_Handler over SWD, bypassing the bootloader:

```sh
openocd -f firmware/openocd.cfg -c "init; reset halt; reg msp [read_memory 0x08004000 32 1]; reg pc [expr {[read_memory 0x08004004 32 1] & ~1}]; reg xPSR 0x01000000; resume; exit"
```

This needs firmware from v1.2.0 on: before that, `startup.c` never enabled the FPU and the apps only ran because the
bootloader left it on (a direct jump HardFaulted with UFSR.NOCP). Always confirm with
`tools/check_hardware_ready.py --boot-check` before debugging further. A standalone power-on (no ST-Link) still depends
on the bootloader.

## Rules

- **No unverified STM32 register values.** Every register field written in `firmware/` is checked against RM0368 or
  the vendored CMSIS header before it is written. If it cannot be verified, the code comment says so. Pin and
  alternate-function mappings cite where they were checked (ST's pin-data XML).
- **Tests on recorded hardware data assert properties, not pinned values.** A test that embeds captured sensor data
  asserts an invariant with headroom (e.g. "spread shrinks by at least 5x"), so it fails when something breaks, not
  when an algorithm is legitimately retuned.
- **Raw data settles disputes.** A threshold, filter or register value is judged against raw data captured on the
  hardware, not against an algorithm's own output or a remembered value.
- **Zero allocation serves the goal, not the reverse.** The goal is smooth, low-cost arm control; trade-offs are judged
  against that.
- **Grip control is uniform-curl, not per-finger.** Per-finger control was tried and dropped: a coverage sweep showed the
  grasp worked at only one pose either way.
- **Documentation claims are checked against the code.** README, PRD and firmware/README state only what the code or a
  dated measurement supports; `tools/test_docs_match_code.py` checks the numbers in CI.

## Target hardware

STM32F401RCT6 Black Pill, 1x MyoWare 2.0 (EMG, PA0/ADC1), 2x MPU6050 (I2C1, PB6/PB7, addresses 0x68/0x69 via AD0).
MeArm with 4x SG92R servos on TIM3 CH1-4 (PA6 base, PA7 shoulder, PB0 elbow, PB1 claw), own 4xAA supply. FT232RL
USB-serial adapter (the CP2102 has a known firmware-lockup bug; see `tools/usb_serial_port.py`).

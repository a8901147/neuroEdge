# neuroEdge — working notes for Claude

An EMG+IMU controlled robotic arm: zero-heap-allocation C++ control loop on
a bare STM32F401RCT6 (Black Pill), driven by a MyoWare 2.0 EMG sensor and
2x MPU6050 IMUs, bridged live into a MuJoCo simulation for validation.

- **What/roadmap**: [PRD.md](PRD.md)
- **Dated history, debugging stories, past findings**: [SESSION_LOG.md](SESSION_LOG.md)
- **How to use the tools/scripts**: [README.md](README.md)

This file only covers things specific to working with Claude on this repo —
conventions, not project state. Don't duplicate PRD/SESSION_LOG/README here.

## Build & test

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

# Python tests (tools/, tools/mujoco_bridge/) — see .github/workflows/ci.yml
# for the exact list this repo's CI actually runs.
```

After any SWD flash, the app will not run until a genuine **power-cycle**
(unplug/replug USB) — a reset alone is not enough. Use
`tools/check_hardware_ready.py --boot-check` to confirm the app is
actually running before debugging further.

## Rules specific to this repo

- **No speculative STM32 register values.** Every register field written in
  `firmware/` must be verified against RM0368 (or the vendored CMSIS header
  in `firmware/build/_deps/cmsis_device_f4-src/Include/`) before it's
  written — grep the header for the exact bit position/macro rather than
  recalling it from memory. If it can't be verified, say so explicitly in
  the code comment and flag it to the user instead of presenting a guess as
  fact. AF/pin-mapping claims should cite where they were checked (ST's
  official pin-data XML, not a blog post) with the same rigor.

- **Real-hardware regression tests assert properties, not pinned values.**
  When a test embeds real captured sensor data (e.g. a tremor-smoothing
  test using real gyro samples), assert an invariant with headroom (e.g.
  "spread shrinks by at least 5x") rather than an exact number matching one
  specific run. Pinning exact values makes the test fight every future
  legitimate retune of the algorithm instead of only failing when
  something is actually broken.

- **Raw data is the arbiter.** When something's correctness is in dispute
  (a threshold, a filter, a register value), settle it with real captured
  sensor data from real hardware — never an algorithm's own derived
  output, and never a value recalled from memory instead of checked.

- **Zero-allocation is a means, not the end goal.** The real target is
  smooth, low-cost arm control. Judge Phase 3+ tradeoffs against that, not
  against strict `malloc_count == 0` dogma for its own sake.

- **Keep grip control uniform-curl, not per-finger.** Already tried and
  rejected as over-engineering after a coverage sweep showed grasp only
  actually worked at one pose regardless — don't re-litigate this without
  new evidence.

- **PR workflow since v1.0.0**: this repo has a tagged stable release.
  Flag it proactively when a unit of work looks like a good PR point —
  don't wait to be asked — but don't open the PR itself unprompted.

## Target hardware (confirmed 2026-08-22)

STM32F401RCT6 Black Pill, 1x MyoWare 2.0 (EMG, PA0/ADC1), 2x MPU6050
(I2C1, PB6/PB7, addresses 0x68/0x69 via AD0). FT232RL USB-serial adapter
(CP2102 has a known firmware-lockup bug — see `tools/usb_serial_port.py`).

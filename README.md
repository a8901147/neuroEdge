# EdgeNeuro

Modular, zero-allocation, real-time BCI/neuroprosthetic signal-processing engine in C++20. See [PRD.md](PRD.md) for the full product/architecture spec. This README covers day-to-day build/test/run commands only.

**Phase 1** (host-side C++20 algorithmic core) is done — validated for `malloc_count == 0` and sub-microsecond per-sample latency. Currently in **Phase 1.5**: a scoped feasibility spike putting the same engine on real STM32F401 hardware before committing to full Phase 3. **Phase 2 iteration 1** (MuJoCo bridge, see below) is also done — see PRD §6 for the quality-gate discipline and Phase 1.5's rationale.

## Layout

```
include/edgeneuro/   Header-only engine: concepts, ring buffer, filters, features, classifiers, providers, pipeline, bump allocator
src/                 no_heap_guard.cpp (allocation counter) + main.cpp (terminal demo) + gui_demo.cpp (ImGui/ImPlot demo) + mujoco_bridge_demo.cpp (Phase 2 MuJoCo bridge)
tests/               Catch2 unit + integration tests
benchmarks/          Google Benchmark suite (<1,6> wearable fusion, <32,0> HD-sEMG stress)
data/                Synthetic CSV fixtures (not real Ninapro data — see tools/generate_sample_data.py)
firmware/            Phase 1.5: STM32F401 feasibility spike (bare CMake + arm-none-eabi-gcc) — see firmware/README.md
tools/mujoco_bridge/ Phase 2: Python + MuJoCo viewer that drives a unitree_g1 left arm+hand from mujoco_bridge_demo.cpp's stdout (or live hardware, see run_demo_live.py)
```

## Build

Four mutually exclusive [CMake Presets](CMakePresets.json), each mapping to one PRD §5 validation row. Don't mix `EDGENEURO_ENABLE_HEAP_GUARD` with a sanitizer in the same binary — ASan/TSan install their own allocator interceptors that conflict with `NoHeapGuard`'s global `operator new` override (CMake will refuse the combination).

| Preset | Purpose |
| --- | --- |
| `debug-heapguard` | Debug build, `NoHeapGuard` armed — proves `malloc_count == 0` |
| `sanitize-asan-ubsan` | ASan + UBSan — undefined behavior, no heap guard |
| `sanitize-tsan` | TSan — RingBuffer concurrency correctness (separate binary; ASan/TSan can't coexist) |
| `release-bench` | Release + `NoHeapGuard` + Google Benchmark |

```sh
cmake --preset debug-heapguard
cmake --build --preset debug-heapguard -j
```

Swap `debug-heapguard` for any other preset name to switch configs.

## Test

```sh
ctest --preset debug-heapguard          # or sanitize-asan-ubsan / sanitize-tsan
ctest --test-dir build/release-bench    # release-bench has no dedicated test preset
```

## Coverage

Source-based coverage via `llvm-cov` (Xcode Command Line Tools on macOS; plain `llvm-profdata`/`llvm-cov` on Linux — drop the `xcrun` prefix there). Separate build dir, not one of the four presets, since it needs its own compiler flags:

```sh
cmake -S . -B build/coverage \
  -DCMAKE_BUILD_TYPE=Debug \
  -DEDGENEURO_ENABLE_HEAP_GUARD=ON \
  -DEDGENEURO_BUILD_BENCHMARKS=OFF \
  -DEDGENEURO_BUILD_GUI_DEMO=OFF \
  -DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping" \
  -DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate"
cmake --build build/coverage -j --target edgeneuro_tests

cd build/coverage
mkdir -p coverage_raw
LLVM_PROFILE_FILE="coverage_raw/edgeneuro-%p.profraw" ./edgeneuro_tests
xcrun llvm-profdata merge -sparse coverage_raw/*.profraw -o coverage.profdata
xcrun llvm-cov report ./edgeneuro_tests \
  -instr-profile=coverage.profdata \
  -ignore-filename-regex='_deps/|/tests/'
```

The `-ignore-filename-regex` scopes the report to our own `include/edgeneuro/` + `src/` — otherwise it'd also count FetchContent'd Catch2/benchmark source and the test files themselves. Swap `report` for `show <path/to/file>` to see line-by-line annotated source for one file. See PRD §5 for the current measured numbers and what's structurally unreachable.

## Benchmark

```sh
cmake --preset release-bench && cmake --build --preset release-bench -j
./build/release-bench/edgeneuro_bench
```

Validates PRD §5's `<32,0>` continuous latency `< 0.1ms` target and `malloc_count == 0` for both `<1,6>` and `<32,0>` modes.

## Demo

Terminal oscilloscope replaying the synthetic `<1,6>` wearable-fusion CSV at a real 1kHz pace — live EMG amplitude bar, decoded gesture, per-tick latency, running `malloc_count`.

```sh
python3 tools/generate_sample_data.py   # regenerate data/*.csv if needed
./build/debug-heapguard/edgeneuro_demo data/wearable_1emg_6imu.csv
```

### Graphical demo (ImGui + ImPlot)

Same engine and CSV, rendered as a real line chart in a native window instead of an ASCII bar. Off by default (pulls in GLFW/OpenGL via FetchContent) — enable with `-DEDGENEURO_BUILD_GUI_DEMO=ON`:

```sh
cmake -S . -B build/gui-demo -DEDGENEURO_BUILD_GUI_DEMO=ON -DEDGENEURO_BUILD_TESTS=OFF -DEDGENEURO_BUILD_BENCHMARKS=OFF
cmake --build build/gui-demo -j
./build/gui-demo/edgeneuro_gui_demo data/wearable_1emg_6imu.csv
```

## MuJoCo bridge demo (Phase 2)

Drives a whole-arm [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) `unitree_g1` left arm+hand (forked from `unitree_g1/g1_with_hands.xml`, full mannequin frozen except the left arm's 14 driven joints — see `tools/mujoco_bridge/arm_hand_scene.xml`'s header comment for why this replaced an earlier hand-tuned Shadow Hand weld) from the same real-hardware-validated control logic as Phase 1.5's firmware (`GripStateMachine` + two `ComplementaryFilter` instances + `SlewRateLimiter`, bypassing `Pipeline`/`EdgeNeuro<>` — see PRD §3's architecture note) — `src/mujoco_bridge_demo.cpp` replays a CSV instead of real ADC/I2C hardware and streams decoded grip/shoulder/elbow state to a Python viewer. Sensor count matches the confirmed real Phase 3 hardware budget (2x MPU6050 + 1x MyoWare) — see PRD §3/§6 for the sensor-to-DOF mapping.

**Grasp status**: `tools/mujoco_bridge/test_grasp_object.py`/`test_myoware_grip_replay.py` verify a real contact-based pickup (reach → close → lift → hold for 10s, checking the object's motion tracks the hand's, not just its absolute height) at the scene's current front-left reach pose, at `GRIP_SCALE=0.57` — both a hand-authored grip ramp and a real EMG-test-data-decoded grip hold there. `test_grasp_coverage.py`'s wider sweep (5 reach directions × 5 grip scales, checked against the real ~5-10s hold the actual 6-step task needs) shows this does **not** generalize across reach direction: front-left holds cleanly across `GRIP_SCALE` in `[0.54, 0.60]` and fails outside that window, but the other 4 tested poses fail at *every* grip scale — either dropping the object outright during the grip phase itself, or never actually picking it up at all. The uniform-curl grip (all 6 finger actuators scaled by the same fraction, not per-finger targets shaped to the object) is the reason, and per the user's explicit direction this is accepted scope, not something to fix with per-finger control — see `test_grasp_coverage.py`'s own docstring for the full breakdown. `test_grip_kinematics.py` checks the hand's open/close range in isolation with no object.

```sh
# One-time setup
brew install python@3.12
python3.12 -m venv .venv
.venv/bin/pip install -r tools/mujoco_bridge/requirements.txt

# mujoco_menagerie is an untracked local dependency (gitignored, not vendored) — sparse checkout:
git clone --no-checkout --depth 1 --filter=blob:none https://github.com/google-deepmind/mujoco_menagerie.git
cd mujoco_menagerie && git sparse-checkout init --cone && git sparse-checkout set unitree_g1
git checkout da76818e269b82289eba39808e2fb91d679d6994 && cd ..

python3 tools/generate_sample_data.py   # regenerate data/*.csv if needed
cmake --build build/debug-heapguard --target edgeneuro_mujoco_bridge_demo
.venv/bin/mjpython tools/mujoco_bridge/run_demo.py   # macOS: must be mjpython, not python3
```

### Live hardware (Stage 6): the actual point, not the CSV replay

`run_demo.py` above replays a synthetic CSV — a prototype used to validate the
3D model/control mapping in isolation. The real goal is the physical device
(STM32F401 + real MyoWare + 2x real MPU6050) decoding and streaming live over
USART2/USB-TTL, driving the same MuJoCo arm in real time. Firmware side:
`firmware/src/phase3_control_loop_main.cpp`'s Stage 6 dual-IMU loop (see
`firmware/README.md` for wiring — two MPU6050s share I2C1 at addresses
`0x68`/`0x69`, `AD0` tied to 3.3V on the second unit). Python
side: `tools/mujoco_bridge/run_demo_live.py`, a separate script from
`run_demo.py` (different lifecycle — a live serial port has no "finished"
sentinel), reusing the same MuJoCo model/actuator wiring, reading over
`pyserial` instead of replaying a subprocess:

```sh
# after flashing phase3_control_loop, start the app with the SWD jump in
# CLAUDE.md (2026-10-03: the WeAct bootloader often won't jump to it on its
# own; replug and `reset run` are unreliable), confirm with
# `tools/check_hardware_ready.py --boot-check`, then
# connect the USB-TTL adapter (FT232RL by default; see tools/usb_serial_port.py)
.venv/bin/mjpython tools/mujoco_bridge/run_demo_live.py
.venv/bin/mjpython tools/mujoco_bridge/run_demo_live.py --cp2102               # use CP2102 instead of auto-detecting
.venv/bin/mjpython tools/mujoco_bridge/run_demo_live.py --port /dev/tty.usbserial-XXXXXXXX  # explicit override
```

Verify hardware bring-up SWD-first before trusting the viewer (see
SESSION_LOG.md for the full bring-up history): `python3 tools/check_hardware_ready.py
--i2c-scan` should ACK both `0x68` and `0x69`; the firmware's
`g_wake_result_shoulder`/`g_wake_result_elbow` globals should both read `0`
via `openocd ... mdw`.

### Diagnostic and calibration utilities

Every script below shares `tools/usb_serial_port.py`'s port auto-detection:
no `--port` needed if only one USB-TTL adapter is plugged in, `--cp2102`
selects CP2102's fixed path explicitly (its own path doesn't change per
unit, unlike FT232RL's), and `--port <path>` overrides either.

| Script | Use it when... |
| --- | --- |
| `python3 tools/check_hardware_ready.py [--i2c-scan] [--sensors] [--live-check] [--boot-check]` | Before trusting anything else in this list — confirms ST-Link/USB-TTL/serial port are all actually usable, optionally the I2C bus + both MPU6050s (`--i2c-scan`), both IMUs' live data from the running firmware without flashing anything (`--sensors`), a full live data flow (`--live-check`), or (no reflash) that execution reached the app (`--boot-check`; it prints the SWD jump command if the board is still in the bootloader). |
| `mjpython tools/mujoco_bridge/run_demo_live.py [--mearm]` | The actual task — full calibration + live MuJoCo control from real hardware. `--skip-calibration --skip-emg-calibration` reuses `shoulder_calibration.json` instead of re-running the pose/EMG calibration flow. |
| `mjpython tools/mujoco_bridge/run_demo.py` | No hardware available, or isolating whether a problem is in the MuJoCo/control-mapping logic itself (replays a CSV instead of live serial). |
| `mjpython tools/mujoco_bridge/calibrate_mearm_alignment.py` | Before trusting `run_demo_live.py --mearm`, or whenever the MeArm model doesn't move like your arm (sensor re-mounted/re-strapped, or lift/elbow/left-right look wrong). Interactive: the MeArm model shows a target pose, you copy it with your real arm, and the captured sensor values become the measured calibration (raw pose vectors + elbow polarity/range) saved into `shoulder_calibration.json` as `mearm_alignment` (old file backed up first). Also asks two y/n questions you answer by looking at the model (does its elbow fold mean your bend? which claw end is closed?). Board must be running `phase3_control_loop`. |
| `mjpython tools/mujoco_bridge/run_demo_live_grip_only.py` | Testing just the MyoWare → grip path in isolation (no IMUs wired up, or ruling out shoulder/elbow tracking as a variable). |
| `python3 tools/capture_arm_motion.py` | Choosing the MEArm's 1-euro filter parameters from real data: walks you through three short recordings of both IMUs (still / slow / normal-speed arm motion; each starts on Enter and can be redone), saves `data/arm_motion_<timestamp>.json` and prints the measured motion speeds and resting noise. Board must be running `phase3_control_loop`. |
| `python3 tools/watch_emg_raw.py` | The EMG signal itself seems off, or before (re)calibrating the grip threshold — watch the live `emg_min`/`emg_max` trace while actually clenching, instead of guessing timing blind. |
| `python3 tools/watch_myoware_uart.py` | Superseded by `watch_emg_raw.py` for current firmware; only useful against the older Stage 3c/3d/5a firmware targets in `firmware/README.md`. |
| `mjpython tools/mujoco_bridge/sensor_orientation_sanity.py` / `sensor_xy_sanity.py` | Suspect a sensor is mounted backwards or wired wrong at a fundamental level — bypasses calibration/oblique-decompose entirely, just "tilt/move the sensor, watch the shape move the same way." |
| `python3 tools/mujoco_bridge/log_raw_imu.py` | Need to (re)capture raw IMU ground-truth data for a fixture (e.g. `test_imu_to_mujoco.py`'s own captured poses) — no MuJoCo viewer needed. |

`tools/generate_sample_data.py` and `tools/convert_epn612.py` (below) are
Stage 1 host-only data-prep tools, not live-hardware utilities — listed
under their own section since real hardware has superseded that workflow.

## Phase 4: MEArm physical actuator bring-up

Firmware targets (`firmware/CMakeLists.txt`, Stage 7a-7d) for driving the
MEArm's 4 servos (base/shoulder/elbow/claw) via TIM3's 4 PWM channels
(PA6/PA7/PB0/PB1), one servo per channel:

| Target | Use it when... |
| --- | --- |
| `servo_pwm_test` | First bring-up of a single servo on PA6 — sweeps 1000-2000us so it can be visually confirmed to move at all, before wiring 4. |
| `servo_limit_finder` | Finding one servo's real safe pulse-width range by ear (UART `+`/`-`, one channel, hardcoded to PA6). |
| `servo_pwm_4ch_test` | All 4 servos wired — moves them one at a time (base→shoulder→elbow→claw) so a miswired channel shows up as "this one didn't move," not 4 moving with no way to tell which is wrong. |
| `servo_limit_finder_4ch` | Same as `servo_limit_finder` but for all 4 mounted servos in one session — UART `1`-`4` selects the channel, `+`/`-` nudges it. Needed because the assembled MEArm's linkage geometry caps each joint's real range well below the bare servo's own limit (community MeArm calibration data: ~90-100° per joint once mounted, vs. one bare SG92R measured here at ~194° free-spinning). |
| `python3 tools/servo_pose_4ch.py` | Finding a safe rest pose by hand, with `servo_limit_finder_4ch` flashed: `set shoulder 1650`, `all 1500`, `show`. Never leaves the measured ranges, always moves 25 µs at a time, prints the final pose as one line. |
| `python3 tools/measure_linkage_region.py` | Measuring the real arm's feasible shoulder×elbow region (SESSION_LOG TODO C), with `servo_limit_finder_4ch` flashed. Moves one servo at a time, one 25 µs step per ~0.8 s; **you press Enter at the first sign of binding** (buzzing, straining, links stopping), it backs off and records the pulse. Asks the cause of each stop (linkage vs. collision). Saves raw results to a new `data/mearm_linkage_<timestamp>.json` after every shoulder position (never overwrites); `--shoulders 1350,1350,1425` picks/repeats positions, `--analyze FILE...` merges runs (no hardware) and shows the repeatability. |
| `python3 tools/gen_mearm_envelope.py` | Regenerates the real arm's safe (shoulder, elbow) pulse envelope — `include/edgeneuro/control/mearm_envelope_data.hpp` and `data/envelope_golden.csv` — from the raw measurement files listed in `SOURCES`. Coarse and conservative: one elbow window per measured shoulder position, windows INTERSECTED between them (never interpolated), shoulder limited to the measured range; every stop of any cause counts. Used by `edgeneuro::PulseEnvelope` (not wired into the firmware yet). |
| `python3 tools/measure_servo_angles.py` | Measuring how the real arm's LINK ANGLES depend on the servo pulses (Path B's missing last step: model command → pulse). Moves one servo at a time inside the safe envelope and asks you to type each link's angle (a phone-inclinometer reading, sign supplied by you: 0 = horizontal, far end higher = positive). Part A: shoulder → upper-arm elevation; part B: elbow → forearm elevation. Saves a new `data/mearm_angles_<timestamp>.json`; `--analyze FILE` (no hardware) fits the two lines, warns about implausible slopes / outliers, and says how much of the model's shoulder travel the arm can reach. Model side: `tools/mujoco_bridge/mearm_pulse_map.py`. |

### Servo <-> STM32 pin assignment

Assigned bottom-to-top by physical position on the MEArm (base is the
bottom-most servo, claw is the top-most/end-effector) — not an arbitrary
channel-number pick:

| Servo | STM32 pin | Timer channel |
| --- | --- | --- |
| Base | PA6 | TIM3_CH1 |
| Shoulder | PA7 | TIM3_CH2 |
| Elbow | PB0 | TIM3_CH3 |
| Claw | PB1 | TIM3_CH4 |

PA6/PA7/PB0/PB1 are confirmed free: PA0 is EMG ADC, PA2/PA3 are USART2,
PB6/PB7 are I2C1 (both IMUs), PC13 is the LED (`phase3_control_loop_main.cpp`).

### Wiring checklist

**Current wiring (2026-10-01, the one that made the servos run smoothly — SESSION_LOG "伺服電源重新接線"):** a 4×AA
battery box (~5 V) powers the servos only; two WAGO 221-415 lever connectors are the + and − distribution points
(battery, the capacitor, all 4 servo red/brown wires via male dupont leads, and on the − one a wire to the Black Pill
GND); a 1000 µF solid capacitor plus a 0.1 µF ceramic sit across the two WAGOs; the battery's thin leads go into the
WAGOs through crimped ferrules (strip ~24 mm and fold so the copper fills the ferrule — pliers are not enough, use a
ferrule crimper). Servo current never goes through the breadboard. **Forgetting the common ground makes the servos not
move at all** even though the board outputs correct PWM. The checklist below is the original (USB-charger) version;
the rules in it (separate supply, common ground, bulk capacitor) still apply.

Servos need far more current (up to ~2.6A worst-case across 4 servos
under stall) than the STM32 board's own USB/5V rail is rated for — powering
them from the same rail as the board risks a brownout that resets the
STM32 when a servo moves. Power is external and separate; only the PWM
signal lines go to the STM32:

- [ ] All 4 servo **signal** wires (orange/yellow) → PA6 / PA7 / PB0 / PB1 respectively (one each, no sharing)
- [ ] All 4 servo **+5V** wires (red), paralleled together → a plain (non-fast-charge/PD) 5V/2A+ USB charger, via a cheap USB-A breakout board (splits a USB socket's VCC/GND out to header pins — only VCC/GND are used, D+/D- ignored)
- [ ] All 4 servo **GND** wires (brown/black), paralleled together → the same charger's GND (via the same breakout board) **and** a wire to the STM32 board's own GND (common ground — without this the PWM signal has no shared reference and won't be read correctly)
- [ ] A **bulk electrolytic capacitor (470-1000uF, 10V+ rating)** in parallel across the shared +5V/GND rail, close to the servos (mind polarity — the stripe/shorter leg is negative). Not optional: a servo's startup current draw is a millisecond-scale spike a simple charger+breakout board can't source fast enough, so voltage sags below what the servo's control chip needs *just for that spike* — too fast for a multimeter to ever show (it'll read a perfectly normal ~5V), but real enough to cause a servo to chatter/twitch/not move at all. Found the hard way on 2026-09-21 (channels 3/4 wired to PB0/PB1 tested completely dead — ruled out miswiring via direct SWD register reads and a known-good servo swap before landing on this) — see `SESSION_LOG.md`'s 2026-09-21 entry for the full trace.
- [ ] STM32 board itself stays powered from its own USB connection (computer or a separate charger) — never the same 5V rail as the servos

One already-measured bare servo (SG92R, single-channel bring-up unit,
unmounted): grinding/mechanical limit found at pulse_us=400 and
pulse_us=2550, center=1475 (not the textbook 1500 — per-unit factory
calibration tolerance, see `servo_pwm_test_main.c`'s 2026-09-21 comment).

Each of the 4 servos' real range once actually mounted on the assembled
MEArm (2026-09-23, `servo_limit_finder_4ch`, written into
`phase3_control_loop_main.cpp`'s `kBasePulseMap`/`kShoulderPulseMap`/
`kElbowPulseMap`/`kClawPulseMap`):

| Servo | Real pulse_us range |
| --- | --- |
| base | 500-2500 (no mechanical stop found within the bare servo's own safe range — see below) |
| shoulder | 1200-2100 |
| elbow | 500-1850 |
| claw | 1300-1600 (full open to full close) |

**`phase3_control_loop` does NOT drive these servos by default (2026-09-26).**
Flashing it to view the MuJoCo model (`run_demo_live.py --mearm`) must not also
move the physical arm, and its servo mapping is not verified yet: shoulder and
elbow are driven independently (the MeArm's linkage couples them — SESSION_LOG
TODO C) and from the firmware's own pitch/roll/elbow values rather than the
calibrated decode `--mearm` uses. To opt in deliberately:

```sh
cd firmware && cmake -S . -B build -DEDGENEURO_DRIVE_SERVOS=ON && cmake --build build --target phase3_control_loop
```

With servos ON, `phase3_control_loop` drives shoulder and elbow from the same Path B decode as `--mearm`
(`edgeneuro::mearm::drive::command`: stretch mode → measured link-angle lines → measured safe envelope → each servo's
slow start-up ramp); **the base holds 1500** (not measured yet) and the claw follows grip. The sensor calibration is
compiled in: after re-calibrating, regenerate it and re-flash:

```sh
python3 tools/mujoco_bridge/gen_calibration_header.py        # shoulder_calibration.json -> mearm_calibration_data.hpp
cd firmware && cmake -S . -B build -DEDGENEURO_DRIVE_SERVOS=ON && cmake --build build --target flash_phase3_control_loop
```

CI checks the default build has no TIM3 reference at all. (The bring-up firmwares
`servo_pwm_test`/`servo_limit_finder*` are unaffected — they exist to drive servos.)
A board flashed BEFORE this change still drives the servos: reflash (and
power-cycle) to get the safe default.

When enabled, each servo starts at its own rest pulse — **1500 µs for base/shoulder/elbow
and 1300 µs (open) for the claw**, picked 2026-09-26 by looking at the real arm, not
measured as "safe" — and walks to the sensor-driven target slowly, then follows
at full speed — see `servo_startup_ramp.hpp`. The very first pulse after power-up
still moves a servo from wherever it was at its own full speed (open loop, no
feedback, and SG92R does not return anywhere when unpowered) — that step cannot be
slowed by firmware. There is no shutdown "park" routine on purpose (depends on
operator habit).

Not assumed identical across units — each was measured individually.
Base's range resolves the earlier `shoulder_roll` concern above (task
needs ~91°, the MeArm community's own ~90° calibration data looked
razor-thin): this specific unit's base joint measured essentially the
full bare-servo range (2000us, close to the unmounted servo's own
2150us), well past what 91° needs — the community's ~90° figure looks
like a conservative *soft limit* in their own calibration, not this
mechanism's true mechanical stop.

Pulse-width **polarity** (which physical end of a channel's pulse range
a more-negative vs. more-positive live sensor value should drive) is a
separate, still-open question — see `phase3_control_loop_main.cpp`'s
2026-09-23 comment above `kShoulderPulseMap` for the reasoning that gives
shoulder's polarity reasonable (but not live-verified) confidence; base/
elbow/claw have no equivalent grounding yet and need a live check before
trusting them.

### Sensor health is checked automatically (v1.1.0)

After a day lost to loose IMU wiring mistaken for algorithm bugs, the tools check both MPU6050s themselves
(`tools/sensor_health.py`), so a hardware fault can't silently look like a wrong model:

- **The data can't be trusted** — no readings, 0 completed I2C reads, **bit-identical ("frozen") readings**, an axis
  stuck at full scale, a magnitude that is not ~1 g, or a sensor reporting itself asleep. `run_demo_live.py` **will not
  start** (it says which sensor and what is wrong, and continues by itself once it is fixed); mid-session the arm
  **holds its last pose** with a warning instead of following bad data and resumes by itself; a calibration capture
  recorded during a fault is redone, never saved.
- **The data is right but the connection is flaky** — the sensor stopped answering on I2C and came back, or reset and was
  woken again. The arm keeps following; a note naming the sensor is printed (again at most every 30 s while it lasts),
  and "✓ 感測器沒有再斷線。" once it stops.
- Sensors declared absent with `--optional-sensors` are not counted as faulty.

The flaky-connection counters come from the hardware itself, not inferred from the data: `phase3_control_loop`'s diag
line reports each sensor's I2C NACKs/timeouts, and once a second it reads each MPU6050's own `PWR_MGMT_1` (0x6B, register
map RM-MPU-6000A-00 Rev 4.0) — a sensor that lost power for a moment between reads comes back with `SLEEP` set and no I2C
error, so the firmware wakes it again and counts it (`shoulder/elbow_pwr_mgmt_1`, `shoulder/elbow_power_resets`).

One-off checks, nothing flashed:

```sh
python3 tools/check_hardware_ready.py --sensors      # reads the running phase3_control_loop for ~3 s
python3 tools/check_hardware_ready.py --i2c-scan     # FAILS unless BOTH 0x68 (upper arm) and 0x69 (forearm) answer
python3 tools/watch_imu_raw.py                       # per-second raw view of both IMUs (--no-verdict for numbers only)
```

A known-good calibration is committed as a golden sample: `data/shoulder_calibration_golden_2026-09-13.json` (the
2026-09-13 real-board calibration, the day the full 6-step grasp task first ran end to end). Use it with
`--skip-calibration --skip-emg-calibration --calibration-file data/shoulder_calibration_golden_2026-09-13.json` to rule
out a bad fresh calibration; it only fits while the sensors are worn/strapped the same way as on that day.

`--sensors` passes when the data is right; a flaky-but-recovering sensor is listed as `[NOTE]` with its name, and a
low data rate (e.g. from repeated dropouts) is only a note — it doesn't affect a demo.

**MeArm path** (`run_demo_live.py --mearm`, `calibrate_mearm_alignment.py`): the same checks -- neither starts on faulty
data, the preview holds the model on a fault, and an alignment capture is redone. On the real arm, `phase3_control_loop`
built with servos ON runs `edgeneuro::ImuHealth` every servo cycle: on a failed IMU **every servo holds its current
pulse** (it stops, it does not move anywhere) and resumes when the data is live again.

### Quick start: posing the servos by hand / measuring the linkage (needs hardware)

Both tools talk to `servo_limit_finder_4ch`, so flash that first
(`cd firmware && cmake --build build --target flash_servo_limit_finder_4ch`), start it with the SWD jump in
`CLAUDE.md` (the bootloader often won't run it on its own), then check with
`python3 tools/check_hardware_ready.py`. Nothing else may have the serial port open
(a stray `miniterm` gives `Resource busy`).

```sh
cd /Users/jeremmy/Desktop/neuroEdge
source .venv/bin/activate
python3 tools/servo_pose_4ch.py            # pose the servos: show / set shoulder 1650 / all 1500 / q
python3 tools/measure_linkage_region.py    # measure the feasible shoulder x elbow region (you press Enter at the first sign of binding)
```

Inside `servo_pose_4ch.py`: `show` reads the four pulses back, `rest` goes to the chosen rest pose
(base/shoulder/elbow 1500, claw 1300), `set <channel> <µs>` walks one servo there 25 µs at a time
(channels: 1-4 or base/shoulder/elbow/claw), `all <µs>` puts base/shoulder/elbow at <µs> and **always
sends the claw to its own rest (1300), never to <µs>**, `+ 4` / `- 2` nudge the selected channel, `q` quits
and prints the final pose as one line (`base=1500 shoulder=1500 elbow=1500 claw=1300`, paste-ready). It
never leaves the measured ranges, and `rest`/`all` move the elbow before the shoulder.
The board boots at the chosen rest pose — base/shoulder/elbow 1500 µs, claw 1300 µs (open) — so a fresh
session starts from the same pose every time. The very first pulse after power-up still moves each servo
from wherever it was, at its own full speed: clear the space around the arm before powering up.

`measure_linkage_region.py` brings the claw to 1300 and the base to 1500 first, starts EVERY shoulder
position from the same rest pose (elbow first, then shoulder — backlash makes a position reached from
elsewhere unrepeatable), is back at rest whenever it asks you something, and puts the whole arm back to
rest when it ends (also on Ctrl+C or an error).

## Real-dataset compatibility (EMG-EPN-612)

`tools/convert_epn612.py` converts [EMG-EPN-612](https://zenodo.org/records/4421500) (Myo armband) per-user JSON recordings into `CsvSignalProvider`'s CSV layout — `EdgeNeuro<8, 10>` (8 EMG channels + accelerometer/gyroscope/quaternion). Verified end-to-end against real downloaded data (not just a synthetic fixture): the converter, and the resulting CSV read back through the real C++ `CsvSignalProvider`, both confirmed correct on `trainingJSON/user1/user1.json` (298,710 rows, values cross-checked between the Python converter's output and the C++ parser's output).

Two things this confirmed about real hardware, not assumptions:

- **EMG and IMU are independently-clocked streams that don't even divide evenly** — one user1 recording had 992 EMG samples against 249 IMU samples (~3.98:1, not a clean 4:1 despite the dataset's documented ~200Hz/~50Hz headline rates). The converter resamples the slower IMU stream up to the EMG sample count with zero-order hold (repeat the last reading), indexed by length ratio rather than an assumed fixed Hz — this is also what real Phase 3 firmware would do in its main loop (run at the EMG ADC's rate, only refresh the IMU reading every Nth tick). The C++ engine itself needed zero changes for this.
- **A large dataset's `CsvSignalProvider` must be heap-allocated, not stack-allocated** — `CsvSignalProvider<float, 8, 10, 300000>` is ~20.6MB (exceeds macOS's default 8MB stack), so construct it via `std::make_unique`/`new`. This is legitimate: Provider construction is documented setup-phase work that happens before `NoHeapGuard` arms, exempt from the zero-allocation guarantee by design — only the hot-path `tick()` loop must stay allocation-free.

```sh
# Real dataset lives outside git (data/raw/ is gitignored — see the .gitignore
# entry) since it's a 5.48GB Zenodo archive. Download/unzip it yourself, then:
python3 tools/convert_epn612.py --inspect data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json
python3 tools/convert_epn612.py --convert data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json --out data/raw/epn612_user1.csv
```

`tests/test_real_data_integration.cpp` turns this into an automated, repeatable check rather than a one-off manual run: it processes the first 20,000 rows of `data/raw/epn612_user1.csv` (bounded on purpose — a `CsvSignalProvider` at the full ~298,710-row scale is tens of MB, and passing one by value through a constructor call still momentarily lives in that constructor's stack frame even when the enclosing object is heap-allocated) through a real `EdgeNeuro<8, 10>` and asserts `malloc_count == 0` for the whole run. It skips gracefully (Catch2 `SKIP()`) if the converted CSV isn't present, so it doesn't break the build for anyone who hasn't downloaded the dataset. Classifier weights here are arbitrary/untrained — this test verifies the zero-allocation guarantee against real-world data statistics, not classification accuracy.

Building this test surfaced a real bug, in the test harness rather than the engine: reading `NoHeapGuard::count()` after other `REQUIRE(...)` statements (rather than immediately when the armed scope closes) intermittently attributed Catch2's own assertion-machinery allocations to the "hot loop," since `record_allocation()` counts every allocation regardless of `armed()` state and only *aborts* when armed — a later, unrelated allocation between the guard closing and the count being read just silently inflates the same shared counter. The root cause was reasoned out from the code, not caught live in a debugger: since `record_allocation()` always aborts on an armed hit and the process never crashed, none of the extra allocations could have happened while armed, which points at something running *after* the guard closes but *before* the count is read — the intervening `REQUIRE(ticks == ...)`. (A debugger session did confirm Catch2's own machinery allocates via our overridden `operator new` — but those particular captured backtraces were static-initialization-time registrations from program startup, unrelated to the specific failure, since the conditional breakpoint meant to isolate armed-only hits didn't actually filter as intended.) Fixed by snapshotting the count into a local variable the instant the guarded block closes, before any other statement can run, and confirmed by 17+ consecutive clean runs (including fresh rebuilds, which is when the failure had shown up) afterward. All `NoHeapGuard`-checking tests follow this pattern now.

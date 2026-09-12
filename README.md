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
# after flashing phase3_control_loop, power-cycle the board's own power
# supply (not just a reset -- see SESSION_LOG.md's 2026-09-11/12 entries:
# this bootloader empirically does not run the new firmware after a plain
# SWD/pin reset, only after power is actually removed and reapplied), then
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
| `python3 tools/check_hardware_ready.py [--i2c-scan] [--live-check] [--boot-check]` | Before trusting anything else in this list — confirms ST-Link/USB-TTL/serial port are all actually usable, optionally the I2C bus + both MPU6050s (`--i2c-scan`), a full live data flow (`--live-check`), or (no reflash) that execution reached the app after a manual power-cycle (`--boot-check`). |
| `mjpython tools/mujoco_bridge/run_demo_live.py` | The actual task — full calibration + live MuJoCo control from real hardware. `--skip-calibration --skip-emg-calibration` reuses `shoulder_calibration.json` instead of re-running the pose/EMG calibration flow. |
| `mjpython tools/mujoco_bridge/run_demo.py` | No hardware available, or isolating whether a problem is in the MuJoCo/control-mapping logic itself (replays a CSV instead of live serial). |
| `mjpython tools/mujoco_bridge/run_demo_live_grip_only.py` | Testing just the MyoWare → grip path in isolation (no IMUs wired up, or ruling out shoulder/elbow tracking as a variable). |
| `python3 tools/watch_emg_raw.py` | The EMG signal itself seems off, or before (re)calibrating the grip threshold — watch the live `emg_min`/`emg_max` trace while actually clenching, instead of guessing timing blind. |
| `python3 tools/watch_myoware_uart.py` | Superseded by `watch_emg_raw.py` for current firmware; only useful against the older Stage 3c/3d/5a firmware targets in `firmware/README.md`. |
| `mjpython tools/mujoco_bridge/sensor_orientation_sanity.py` / `sensor_xy_sanity.py` | Suspect a sensor is mounted backwards or wired wrong at a fundamental level — bypasses calibration/oblique-decompose entirely, just "tilt/move the sensor, watch the shape move the same way." |
| `python3 tools/mujoco_bridge/log_raw_imu.py` | Need to (re)capture raw IMU ground-truth data for a fixture (e.g. `test_imu_to_mujoco.py`'s own captured poses) — no MuJoCo viewer needed. |

`tools/generate_sample_data.py` and `tools/convert_epn612.py` (below) are
Stage 1 host-only data-prep tools, not live-hardware utilities — listed
under their own section since real hardware has superseded that workflow.

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

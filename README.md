# EdgeNeuro

Low-cost, intuitive control of a robotic arm from a person's own arm motion and muscle activity: a testbed for the
problems that keep myoelectric prosthetic arms expensive and hard to use.

## Why

Myoelectric prosthetic arms are costly, and a large share of users abandon them (Biddiss & Chau, 2007). The control
itself is part of the problem. Users switch modes to move one joint at a time, grips drop or fire when the arm moves,
and multi-gesture schemes need many EMG channels, per-user training, and often a PC. EdgeNeuro addresses these with
inexpensive, off-the-shelf parts:

- **Natural, simultaneous control.** Two IMUs read where the arm points and how far the elbow bends; one EMG channel
  opens and closes the grip. No mode switching and no gesture training.
- **Control on a microcontroller.** For the physical arm, a low-cost Cortex-M4 (STM32F401) does all the processing:
  EMG sampled at 1 kHz, servo commands updated at 100 Hz, in deterministic C++20 with no heap allocation and no HAL.
  (The MuJoCo models are computed on the PC from the data the board streams.)
- **Built for reliability.** A two-threshold grip holds while the arm moves; a faulty or disconnected IMU makes the
  arm hold still instead of moving on bad data. (The EMG channel has no such check yet.)
- **Measured, not assumed.** Loop rate, memory, CPU load and failure modes are measured on the real hardware, including
  the limits that remain. End-to-end latency (arm motion → servo motion) has not been measured yet.

**Scope.** This is a research platform: an able-bodied person controls a desktop 4-servo arm (MeArm) and two MuJoCo
models (a MeArm model and a Unitree G1 humanoid arm and hand). It has not been tested with prosthesis users.

**Demo task:** arm hanging → reach forward → swing left → grip a roll of tape → lift → swing right → place. The full
task has been completed end to end on the physical arm with a real roll of tape, controlled live by one person wearing
the sensors (recorded on video, 2026-10-09). It is one tuned task with one user, not a measured success rate.

```
 MyoWare 2.0 (EMG, ENV) ──ADC 1 kHz──┐
 MPU6050 upper arm (0x68) ──I2C1─────┤   STM32F401 Black Pill, 16 MHz       USART2 115200
 MPU6050 forearm  (0x69) ──I2C1─────┤   1 kHz main loop, zero heap  ───────────────────▶ PC: run_demo_live.py
                                    │                                                    ├─ MuJoCo MeArm model
                                    └──▶ TIM3 PWM 50 Hz ──▶ MeArm base / shoulder /      └─ MuJoCo Unitree G1 arm+hand
                                                             elbow / claw servos
```

## What it does

On the physical arm, computed on the STM32 (the MuJoCo models use different mappings; see
[PRD.md §5](PRD.md#5-control-design)). Only the IMUs' accelerometers (gravity direction) are used for control; the
gyroscopes are read, but no control uses them. Servo commands are updated every 10th tick (100 Hz):

| Part | How |
| --- | --- |
| Grip | MyoWare envelope → EMA → two-threshold (hysteresis) state machine with a 150 ms on/off debounce → slew-rate limit → claw. The PC calibrates the thresholds (relaxed mean + K·std; release halfway back down) and sends them to the board over UART. |
| Arm direction | The upper-arm gravity vector is decoded against four calibrated poses (hang, forward, left, right) into tilt and azimuth ("Path B"). Azimuth drives the base; a slow-follow rule keeps the base steady while the arm is being raised. |
| Height and reach | Raising the arm lowers the elbow servo (claw up); bending the elbow raises the shoulder servo (reach). Every command, including the poses in between, stays inside a safe shoulder × elbow envelope measured on the real arm. |
| Elbow bend | Angle between the two IMUs' gravity vectors (dot product), which avoids the ±90° singularity of Euler-angle differences. |
| Smoothing | MPU6050 DLPF at 5 Hz, then a 1€ filter (0.5 Hz min cutoff, β 1.5) on the servo path. The servos themselves run at full speed. |
| Safety | On the board, a reading that is implausible (an axis at full scale, or a magnitude outside 0.3–3 g) or unchanged for 0.3 s makes every servo hold its pulse; a sensor that resets is woken again. On the PC, missing data and a magnitude far from 1 g are also caught, and the model holds its pose. Slow start-up ramp; an `R` command walks the arm back to a known start pose. Servo outputs are compiled out unless explicitly enabled. |

## Measured results

| What | Result |
| --- | --- |
| Main loop | 1 kHz ADC sampling triggered by TIM2 (1007 ticks/s measured over 30 s; the internal HSI clock runs +0.9 %) |
| IMU reads | Non-blocking I2C state machine on a 100 kHz bus: ~274 completed reads/s per IMU with both IMUs |
| Heap | No `malloc`/`free`/`new`/`_sbrk` symbols in the firmware image; `malloc_count == 0` asserted on the host and on the target |
| Firmware size | `phase3_control_loop` with servos on: 18.4 KB of code, 3.3 KB of static RAM (of 240 KB / 64 KB) |
| Tests | 248 Catch2 test cases (98.7 % line, 88.6 % branch coverage of `include/` + `src/`); 569 Python test functions |

Per-sample latency of the generic `EdgeNeuro<>` pipeline, on synthetic input. Both machines run the same engine code
(`include/edgeneuro/bench/latency_configs.hpp`). The arm's control path does not use this pipeline (PRD §3); its
real-time evidence is the main loop holding 1 kHz above.

| Mode | Apple M1 (Release) | STM32F401 @ 16 MHz, mean | STM32: the tick that classifies (every 50th) | CPU at 1 kHz |
| --- | --- | --- | --- | --- |
| `<1,6>`: 1 EMG + 6 IMU channels, IIR, MAV, LDA (3 classes) | ~7.5 ns | 15.0 µs | 317 µs | 1.5 % |
| `<32,0>`: 32-channel HD-sEMG stress test, LDA (4 classes) | ~39 ns | 134 µs | 1.54 ms | 13.4 % |

At 16 MHz the 32-channel classify tick overruns a 1 ms sample period, while the 1-EMG configuration fits easily. The
clock has not been raised to the chip's 84 MHz.

**Known limitations**
- As the electrodes are placed now, the EMG picks up the muscles that lift the arm: a relaxed hand moving through the
  demo reached 3848 ADC counts, a firm grip held still only 1318. No threshold can separate the two; the fix is moving
  the electrodes onto the finger flexors.
- An accelerometer cannot sense rotation about gravity, so the base direction comes from upper-arm twist, and the twist
  that naturally comes with raising the arm can't be told apart from a deliberate one.
- The EMG channel is not health-checked: a loose electrode can open or close the claw.
- In MuJoCo, the humanoid hand's uniform-curl grasp holds the object only at the front-left reach pose
  (`tools/mujoco_bridge/test_grasp_coverage.py`).

## Repository layout

```
include/edgeneuro/   Header-only C++20 library, shared by the host and the firmware
  pipeline.hpp ...   Generic EdgeNeuro<> engine: concepts, IIR/MAV/RMS, LDA, CSV provider; a standalone SPSC ring buffer
  control/           Real-arm control: grip state machine, Path B decode, MeArm drive, envelope, ramps, health
  filters/, fusion/  1€ filter, complementary filter, tilt/azimuth
firmware/            Bare-metal STM32F401 targets (CMake + arm-none-eabi-gcc), see firmware/README.md
src/                 Host demos: terminal and ImGui oscilloscopes, CSV-replay MuJoCo bridge
tests/               Catch2 tests (host)
benchmarks/          Google Benchmark latency suite
tools/               Python: hardware checks, interactive calibration and measurement tools
tools/mujoco_bridge/ Python: live MuJoCo bridge, MeArm model, Path B / real-arm mapping
data/                Recorded real-hardware data (linkage, link angles, arm motion, golden calibration), synthetic fixtures
docs/                Reference images
```

## Build and test (host)

Four CMake presets, one per validation goal. A heap guard and a sanitizer can't share a binary, since both hook the
allocator.

| Preset | Purpose |
| --- | --- |
| `debug-heapguard` | Debug + `NoHeapGuard`: proves `malloc_count == 0` in the hot loop |
| `sanitize-asan-ubsan` | AddressSanitizer + UndefinedBehaviorSanitizer |
| `sanitize-tsan` | ThreadSanitizer for the lock-free ring buffer |
| `release-bench` | Release + Google Benchmark |

```sh
cmake --preset debug-heapguard && cmake --build --preset debug-heapguard -j
ctest --preset debug-heapguard --output-on-failure

cmake --preset release-bench && cmake --build --preset release-bench -j
./build/release-bench/edgeneuro_bench
```

CI (`.github/workflows/ci.yml`) runs the heap-guard and ASan/UBSan tests, builds every firmware target, checks that the
default firmware build contains no servo output, and runs the Python tests headless.

<details>
<summary>Coverage</summary>

```sh
cmake -S . -B build/coverage -DCMAKE_BUILD_TYPE=Debug -DEDGENEURO_ENABLE_HEAP_GUARD=ON \
  -DEDGENEURO_BUILD_BENCHMARKS=OFF -DEDGENEURO_BUILD_GUI_DEMO=OFF \
  -DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping" -DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate"
cmake --build build/coverage -j --target edgeneuro_tests
cd build/coverage && LLVM_PROFILE_FILE="raw/%p.profraw" ./edgeneuro_tests
xcrun llvm-profdata merge -sparse raw/*.profraw -o cov.profdata
xcrun llvm-cov report ./edgeneuro_tests -instr-profile=cov.profdata -ignore-filename-regex='_deps/|/tests/'
```

On Linux, drop `xcrun`.
</details>

## Running on the hardware

Wiring, toolchain setup and every firmware target are in [firmware/README.md](firmware/README.md).

```sh
# 1. Build and flash the control loop with servo outputs on, then start it
cd firmware && cmake -S . -B build-servos -DEDGENEURO_DRIVE_SERVOS=ON
cmake --build build-servos --target flash_phase3_control_loop
openocd -f openocd.cfg -c "init; reset halt; reg msp [read_memory 0x08004000 32 1]; reg pc [expr {[read_memory 0x08004004 32 1] & ~1}]; reg xPSR 0x01000000; resume; exit"
cd .. && python3 tools/check_hardware_ready.py --boot-check

# 2. One-time Python + MuJoCo setup (on macOS the MuJoCo viewer needs mjpython)
python3.12 -m venv .venv && .venv/bin/pip install -r tools/mujoco_bridge/requirements.txt
git clone --no-checkout --depth 1 --filter=blob:none https://github.com/google-deepmind/mujoco_menagerie.git
(cd mujoco_menagerie && git sparse-checkout init --cone && git sparse-checkout set unitree_g1 \
   && git checkout da76818e269b82289eba39808e2fb91d679d6994)

# 3. Run: sensor check → EMG + pose calibration → calibration sent to the board → live control
.venv/bin/mjpython tools/mujoco_bridge/run_demo_live.py --mearm      # MeArm model + real arm (first homes it with R)
.venv/bin/mjpython tools/mujoco_bridge/run_demo_live.py              # Unitree G1 humanoid arm
```

After an SWD flash the WeAct bootloader often doesn't hand over to the app, so the `openocd` line jumps straight to the
app's reset handler. `--skip-calibration --skip-emg-calibration` reuses the last saved calibration;
`--calibration-file data/shoulder_calibration_golden_2026-09-13.json` uses the committed reference one.

### Tools

| Tool | Use it to |
| --- | --- |
| `tools/check_hardware_ready.py` | Check the ST-Link, the USB-serial adapter and the board first. `--boot-check`: did the app start? `--sensors`: is both IMUs' live data healthy? `--i2c-scan`: do 0x68 and 0x69 answer? (It flashes a scan firmware; flash `phase3_control_loop` back afterwards.) |
| `tools/watch_imu_raw.py`, `tools/watch_emg_raw.py` | Watch raw IMU or EMG data live, unprocessed. |
| `tools/capture_arm_motion.py [--set base_raise\|emg]` | Record real arm motion (and EMG) around the demo task, to tune filters from data. |
| `tools/measure_servo_response.py` | Record the pulses the board actually sends to the servos (TIM3 CCR1–4 over SWD), phase by phase. |
| `tools/servo_pose_4ch.py`, `tools/measure_linkage_region.py`, `tools/measure_servo_angles.py` | Pose the servos by hand, measure the arm's safe shoulder × elbow envelope, and measure link angles against pulse width (with `servo_limit_finder_4ch` flashed). |
| `tools/gen_mearm_envelope.py`, `tools/mujoco_bridge/gen_*.py` | Regenerate the C++ data headers and golden tables from the recorded measurements. |
| `tools/mujoco_bridge/calibrate_mearm_alignment.py`, `view_mearm.py` | Align the MeArm model with the real sensors, or inspect the model alone. |
| `tools/mujoco_bridge/run_demo.py` | Replay a synthetic CSV into the G1 model with no hardware, to check the MuJoCo scene. It uses an older host-side version of the logic (complementary-filter shoulder decode, single grip threshold), not the live algorithm. Needs the `edgeneuro_mujoco_bridge_demo` target built. |
| `tools/convert_epn612.py` | Convert the public EMG-EPN-612 dataset into the CSV format the C++ engine reads. |

### Host demos, no hardware

```sh
python3 tools/generate_sample_data.py                                    # synthetic CSV fixtures
./build/debug-heapguard/edgeneuro_demo data/wearable_1emg_6imu.csv       # terminal oscilloscope
cmake -S . -B build/gui -DEDGENEURO_BUILD_GUI_DEMO=ON -DEDGENEURO_BUILD_TESTS=OFF -DEDGENEURO_BUILD_BENCHMARKS=OFF
cmake --build build/gui -j && ./build/gui/edgeneuro_gui_demo data/wearable_1emg_6imu.csv   # ImGui + ImPlot
```

If the converted EMG-EPN-612 CSV is present under `data/raw/`, `tests/test_real_data_integration.cpp` also runs the
engine over 20,000 rows of real recorded EMG with `malloc_count == 0` (it skips otherwise).

## Documents

- [PRD.md](PRD.md): design, requirements and how each one is validated
- [firmware/README.md](firmware/README.md): firmware targets, wiring, UART protocol
- [CHANGELOG.md](CHANGELOG.md): releases
- [SESSION_LOG.md](SESSION_LOG.md): development history and debugging findings, by date

## References

- Biddiss, E. & Chau, T. (2007). Upper limb prosthesis use and abandonment: a survey of the last 25 years.
  *Prosthetics and Orthotics International*, 31(3), 236–257.
- Farrell, T. R. & Weir, R. F. (2007). The optimal controller delay for myoelectric prostheses. *IEEE Transactions on
  Neural Systems and Rehabilitation Engineering*, 15(1), 111–118.

## License

[MIT](LICENSE)

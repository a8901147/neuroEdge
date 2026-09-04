# Product Requirements Document (PRD)

**Project Name:** EdgeNeuro: Modular & Zero-Allocation Real-Time BCI Engine
**Author:** Chang-Jui Tseng
**Date:** August 2026

## 1. Summary

EdgeNeuro is a real-time C++20 signal-processing pipeline for EMG and IMU data, aimed at prosthetic control on a microcontroller. Its stages are chosen at compile time (C++20 concepts and templates), so the pipeline allocates no memory at run time and needs no virtual calls. Development goes step by step: first the engine on a PC, then a MuJoCo simulation, then the STM32 hardware.

## 2. Problem

Three gaps make prosthetic control algorithms hard to move onto embedded hardware:

1. **Memory.** A microcontroller without an operating system cannot afford garbage collection or heap allocation in the control loop.
2. **Modularity costs speed.** Swapping algorithms through virtual functions adds indirection; firmware therefore usually hard-codes them.
3. **Hard to reproduce.** Trying an algorithm usually needs a large local setup and gives no end-to-end view.

## 3. Architecture

Each stage is a C++20 concept, resolved at compile time. The value type (`float`, `int32_t`) and the channel counts are template parameters, `template <size_t EmgChannels, size_t ImuChannels>`:
* **Wearable mode, `EdgeNeuro<1, 6>`:** one EMG channel (grip) and a 6-axis IMU (arm orientation).
* **Stress mode, `EdgeNeuro<32, 0>`:** 32 EMG channels, to test the zero-allocation design at scale.

> **Control architecture for Phase 3 (2026-08-17): EMG and IMU do not share one `classifier_.classify()` decision.** Mode A above, which feeds all channels into one classifier, shows that `EdgeNeuro<>` is generic; it is not the control logic for the arm. Arm orientation must follow the IMUs continuously (a 200-sample window at 1 kHz would add 200 ms), while the grip is a discrete event decided over a short time. **Phase 3 therefore runs two independent paths:** the IMUs bypass `Pipeline` (`ComplementaryFilter` → orientation), and the EMG goes through a grip decision. `ComplementaryFilter` does not need an `ImuFilterT` slot.
>
> **The EMG path uses a threshold state machine, not `LdaClassifier` (2026-08-17):** Phase 3 has one MyoWare 2.0 on its `ENV` output, already rectified and low-passed in hardware. One channel carries one dimension (how hard the muscle contracts), so a multi-class classifier adds only window latency. A threshold with on and off durations, the usual approach in commercial myoelectric hands, reacts in 50–150 ms. `LdaClassifier` and `Pipeline` stay for a later multi-channel setup.
>
> **Two IMUs for Phase 3 (2026-08-22):** one MPU6050 on the upper arm and one on the forearm. The upper-arm roll/pitch drives the shoulder (2 DOF); forearm pitch minus upper-arm pitch drives the elbow (1 DOF). Both are 6-axis, so yaw stays unobservable, and the elbow formula holds only for motion roughly in one plane. Verified in the MuJoCo simulation (Phase 2), not yet on the hardware.

| Layer | Role | Requirement |
| --- | --- | --- |
| **Provider concept** | Supplies samples. | The host (`CsvSignalProvider`) and the target (`Stm32AdcProvider`) plug in through the same interface. |
| **Pipeline core** | Moves data, owns memory. | A lock-free SPSC ring buffer on `std::array` (no `std::vector` in the loop), for handing samples from an interrupt to the main loop. Tested on the host with two threads under TSan. |
| **Filter concept** | Pre-processing. | `process()` and `reset()`; separate filters for EMG and IMU channels. IIR (direct form II) and pass-through to start. |
| **Feature concept** | Time-domain features. | MAV (mean absolute value) and RMS to start. |
| **Classifier concept** | Intent decoding. | LDA with weights loaded at start-up. |

## 4. Functional requirements

### 4.1 C++ engine

* **Swappable stages:** changing one `using Engine = Pipeline<FilterA, FeatureB>` line changes the algorithm.
* **Deterministic memory:** no heap allocation after start-up.

### 4.2 Recorded and synthetic signal streams

* A CSV reader for EMG and IMU streams. The bundled test data are synthetic (`tools/generate_sample_data.py`, alternating contraction and rest), because Ninapro data may not be redistributed; the reader accepts real Ninapro CSV exports unchanged.
* **Fused stream:** `1-Ch EMG + 6-Axis IMU`, one sample every 1 ms (1000 Hz).
* **Stress stream:** `32-Ch EMG`, for the benchmarks.
* **Real recordings:** the public [EMG-EPN-612](https://zenodo.org/records/4421500) dataset (Myo armband: 8 EMG channels plus 10 IMU values) runs through `CsvSignalProvider` after conversion with `tools/convert_epn612.py`; one user's 298,710 rows match between the Python output and the C++ reader. EMG and IMU arrive on independent clocks (992 EMG to 249 IMU samples, about 3.98:1), as they will from an ADC and an I2C sensor; the slower IMU stream is held between its samples (zero-order hold), with no change to the engine. Classification accuracy is not evaluated yet.

### 4.3 Visualization and MuJoCo simulation

* **Oscilloscope:** a terminal view with no dependencies, and an optional ImGui + ImPlot window (`EDGENEURO_BUILD_GUI_DEMO=ON`, fetched with CMake FetchContent). Both run the same engine on the same stream.
* **MuJoCo:** once the engine is stable, the decoded output drives a simulated hand in MuJoCo.



## 5. Validation


| Check | Tool | Pass criterion |
| --- | --- | --- |
| **Unit tests** | `Catch2` + `llvm-cov` | Over 90 % coverage of the core math and concept interfaces. **Measured:** 46 test cases over `include/edgeneuro/`; line 95.56 %, branch 90.32 % (re-measured 2026-08-12 with `fusion/complementary_filter.hpp`, 100 % covered by 7 new test cases, one of them on real EMG-EPN-612 data). The uncovered lines are the `bad_alloc` path, an unused sized-delete overload, and the `NoHeapGuard` abort path, which `test_no_heap_guard.cpp` checks in a forked child. Measuring coverage found two real bugs: `CsvSignalProvider` silently truncated rows with extra columns, and the firmware's bump allocator returned unaligned memory and threw where no unwind tables exist. Both fixed and tested. |
| **No allocation, no undefined behaviour** | Sanitizers + `NoHeapGuard` | ASan, UBSan and TSan, plus a test that `malloc_count == 0` in the loop. ASan and TSan cannot share a binary and the allocation hook conflicts with them, so each runs from its own CMake preset (`debug-heapguard`, `sanitize-asan-ubsan`, `sanitize-tsan`, `release-bench`). |
| **Latency** | `Google Benchmark` | `<1, 6>` and `<32, 0>`: under 0.1 ms per sample with 32 channels, `malloc_count == 0`. |

## 6. Roadmap

The engine is built and verified on the PC first. Phase 1 passed its checks (zero allocation, latency, sanitizers) in August 2026.

**Revised order (August 2026):** `malloc_count == 0` on the host does not show that the STM32F401 build allocates nothing, fits in its SRAM, or gets a usable EMG signal; MuJoCo cannot answer those questions. A short hardware feasibility check (Phase 1.5) therefore comes before the MuJoCo work.

```
+-----------------------------------------------------------------------------------+
| Phase 1: engine on the PC (done)                                                 |
| C++20 concepts, lock-free ring buffer, zero allocation                           |
+-----------------------------------------------------------------------------------+
                                        |
                         Phase 1 verified
                                        v
+-----------------------------------------------------------------------------------+
| Phase 1.5: hardware feasibility check (current)                                  |
| Zero allocation, memory footprint and EMG quality on the target; no actuation    |
+-----------------------------------------------------------------------------------+
                                        |
                         its results set the priorities of Phase 2 and Phase 3
                                        v
+-----------------------------------------------------------------------------------+
| Phase 2: MuJoCo simulation                                                       |
| The decoded output drives a simulated hand                                        |
+-----------------------------------------------------------------------------------+
                                        |
                         simulation verified before hardware
                                        v
+-----------------------------------------------------------------------------------+
| Phase 3 and 4: STM32                                                             |
| Firmware on the microcontroller, then actuation                                  |
+-----------------------------------------------------------------------------------+
```

### Stage 1: Host-side engine and MuJoCo simulation (no hardware)
No hardware is needed: the engine, the tests and the simulation all run on a PC.

* **Phase 1: engine (C++20, PC) — done (results in §5)**
  * C++20 concepts for `SignalProvider`, `Filter`, `Feature`, `Classifier`, and a `ValueType` parameter.
  * A lock-free ring buffer on `std::array`.
  * `CsvSignalProvider` streams recorded data as if it came from a sensor; checked with the EMG-EPN-612 recordings.
  * `Catch2` tests; ASan, UBSan, TSan and `NoHeapGuard` (`malloc_count == 0`); `Google Benchmark`: 7.10 ns per tick for `<1,6>`, 38.9 ns for `<32,0>`. All four presets pass; line coverage 95.38 %.
  * A terminal view (`src/main.cpp`) and an ImGui + ImPlot window (`src/gui_demo.cpp`) show the signals and the allocation count.

* **Phase 1.5: hardware feasibility check — current**
  * Target: **STM32F401RCT6 Black Pill** (Cortex-M4F, 256 KB Flash, 64 KB SRAM, single-precision FPU, matching `ValueType = float`) and one **MyoWare 2.0** (one channel, open/close only).
  * Deliberately narrow (no IMU, PWM or closed loop yet). It answers four questions:
    1. ✅ `NoHeapGuard` sees `malloc_count == 0` on the board with the `arm-none-eabi-gcc` build. **Passed on the STM32F401, 2026-08-14** (Stage 2 below).
    2. ✅ The pipeline fits in 64 KB SRAM / 256 KB Flash (`arm-none-eabi-size`), with a wide margin.
    3. ✅ A timer-triggered ADC samples at 1 kHz. **Passed on the STM32F401, 2026-08-15** (Stage 3c/3d below).
    4. Is the MyoWare signal good enough? **Conditionally passed (2026-08-15, Stage 3e):** the signal follows real contractions, but the range is narrow; gain is adjusted later.
  * Toolchain: plain CMake + `arm-none-eabi-gcc` in `firmware/`, no STM32CubeIDE. Diagnostics through the LED and UART.

  * **Hardware on hand:** the Black Pill, one MyoWare 2.0, an ST-Link V2, a USB-serial adapter and MPU6050 (GY-521) modules (bought 2026-08-15, not yet wired). Servos and the arm come with Phase 3.

  * **Firmware (`firmware/`):**
    - `cmake/arm-none-eabi-toolchain.cmake`: Cortex-M4F flags (`-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`).
    - `linker/STM32F401RCTx_FLASH.ld`: Flash starts at **`0x08004000`**, 240 KB; RAM 64 KB at `0x20000000`. The board ships with the WeAct HID bootloader in the first 16 KB, which an app at `0x08000000` would overwrite.
    - `src/startup.c`: a hand-written vector table and `Reset_Handler` (`.data`/`.bss` set-up, exceptions to `Default_Handler`). It also points `SCB->VTOR` at the app's vector table, which interrupts will need.
    - `src/main.c`: blink on PC13 (active low) at the default 16 MHz HSI.
    - `src/footprint_check_main.cpp`: builds a real `EdgeNeuro<1,6,...>` from the unmodified `include/edgeneuro/` headers with a stub provider, to measure Flash and SRAM.
    - `CMakeLists.txt`: fetches `cmsis_device_f4` (v2.6.9) and `cmsis-core` (v5.9.0) for the register headers.

  * **Toolchain notes:**
    1. Homebrew's `arm-none-eabi-gcc` has no newlib, so `<stdint.h>` fails. Arm's own release (15.2.rel1) includes it; it is unpacked into `~/.local/arm-toolchain`, which `firmware/cmake/arm-none-eabi-toolchain.cmake` points at.
    2. In boot mode the board enumerates as the WeAct HID bootloader (VID `0x0483`, PID `0x572A`), not as standard DFU, so `dfu-util` does not apply.
    3. The WeAct `hid-flash` tool does not work on Apple Silicon (`hid_enumerate()` returns empty device paths). Flashing goes through an ST-Link and OpenOCD instead.

  * **Stages 0 and 2 pass on the board (2026-08-14, flashed over ST-Link):**
    - **SWD link:** OpenOCD finds the ST-Link V2, 3.28 V target voltage and a Cortex-M4 r0p1.
    - **Stage 0 (blink):** `flash_blink` programs and verifies (device id `0x00016423`, 256 KiB); the PC13 LED blinks at ~1 Hz.
    - **Stage 2 (`NoHeapGuard`):** after `flash_heap_guard_check` the LED stays solid: 100,000 guarded `tick()` calls with `malloc_count == 0` on the ARM build. Question 1 answered.
    - Question 2 (footprint): `text=932 bytes` for Stage 1 and `text=844 bytes` for Stage 2, far below the limits.
    - **Stage 1 (footprint_check):** the unmodified `include/edgeneuro/` headers build a real `EdgeNeuro<1,6,50,...>` for the STM32F401: `text=932 bytes, data=0, bss=0`. Built, not yet flashed.
    - **Flashing:** `firmware/openocd.cfg` (ST-Link, `stm32f4x`, 1 MHz); each `flash_<name>` target programs at `0x08004000`, leaving the bootloader in sector 0 untouched.

  * **The four questions:** 1–3 answered, 4 conditionally passed. Phase 1.5 has done its job.
  * **Stage 4a: I2C1 + MPU6050 (GY-521), passes on the board (2026-08-15).** I2C1 on PB6/PB7 at 100 kHz (RM0368 ch. 18); MPU6050 registers (`PWR_MGMT_1` 0x6B, `WHO_AM_I` 0x75, 14 bytes from `ACCEL_XOUT_H`) and scales (16384 LSB/g, 131 LSB/(°/s)) from InvenSense RM-MPU-6000A-00 and PS-MPU-6000A-00. `firmware/src/i2c_mpu6050_hello_main.c`.

    Four separate problems were found and fixed on the way:
    1. **Wiring:** the USB-serial adapter was on A1/A2 instead of A2/A3.
    2. **Loose module:** a module not fully seated in the breadboard held the I2C bus `BUSY` (read over SWD).
    3. **`WHO_AM_I` = `0x72`**, not `0x68`, on every read: a compatible chip; the firmware accepts both.
    4. **Firmware bug in multi-byte reads:** an N > 2 byte receive must time ACK and STOP with `BTF`, not `RXNE` (RM0368 §18.3.3). Fixed.

    **Debugging over SWD:** reading registers and memory through the ST-Link (`openocd`, `mdw`, `nm`/`addr2line`, diagnostic globals) proved far more reliable than UART output or the LED.

    **Result:** at rest `accel_z ≈ 16850` (≈1.03 g), `accel_x`/`accel_y` near 0, gyro offsets of a few deg/s: the I2C + MPU6050 path works.

  * **Stage 4b blocked: the MPU6050 modules appear faulty (2026-08-17, open).** With `ComplementaryFilter` on the board (`firmware/src/complementary_filter_hello_main.cpp`), I2C fails again and the module never returns to its Stage 4a state. Each step below was checked over SWD:
    1. `BUSY` stuck at 1 → `SWRST` (RM0368 18.6.1) clears it, but the address is then not acknowledged.
    2. New jumper wires → no change.
    3. Different breadboard holes → no change.
    4. Continuity and supply checked with a meter (3.3 V, no shorts) → all fine.
    5. Addresses `0x68` and `0x69` → both NACK.
    6. 100 kHz and 10 kHz → same.
    7. **A second, newly bought GY-521 → exactly the same.**
    8. `SR1` at the failure is `0x400` (AF only): a real NACK, not a firmware misread.
    9. Alternate pins `PB8`/`PB9` (still I2C1) → same.
    10. I2C2 on `PB10`/`PB3` → a different failure, but `PB3` doubles as a debug pin, so inconclusive.
    11. **Decisive test:** an I2C bus scanner with a different device, an LCD1602 backpack (PCF8574): only `0x27` answers and the LCD works. The STM32 side is healthy; the fault is in the two GY-521 modules.
    - **Conclusion:** the I2C1 driver is verified against a third-party device; the `ComplementaryFilter` integration waits for a working MPU6050.

  * **Stress test while waiting for a new MPU6050 (2026-08-17):** `firmware/src/complementary_filter_stress_test_main.cpp` runs the filter loop (math, UART output, an I2C write to the LCD1602 at `0x27` each pass) on smooth synthetic data. 60 s, 367 passes: steady output, no NaN or Inf. The loop period measured 163.4 ms, not the 250 ms assumed from `delay(400000u)`, so `kDt` in `complementary_filter_hello_main.cpp` is now `0.163f`, to be re-measured with a real MPU6050.

  * **Next:** with a working MPU6050, run `ComplementaryFilter` on real data and re-measure the loop period.

  * **2026-08-18: 5 V supply does not help either.** With a single supply (firmware/README.md) and the module on 5 V, both GY-521 modules still do not answer, while the LCD1602 still works on the same wiring. The modules are faulty. An Adafruit MPU-6050 is ordered.

  * **Stage 5a: EMG grip control, passes on the board (2026-08-18).** `firmware/src/emg_grip_control_main.cpp` reads the MyoWare `ENV` at 1 kHz (TIM2 + ADC1, Stage 3c/3d), feeds `GripStateMachine::update()` with `dt=0.001f` and `SlewRateLimiter`, and reports each transition and a once-a-second min/max over USART2. First run on a real signal.
    - **Result:** with real electrodes, relaxed reads **~434–495** and a sustained clench **~3700+**. The fixed `threshold=1220` sits between them with a wide margin. Transitions are clean: on relaxing the signal decays (3717 → 2069 → 1477 → 887 → 484) and `EDGE -> Released` comes only once it stays below the threshold.
    - This also closes question 4: the narrow range seen in Stage 3e came from touching the electrodes with a finger, not from real electrodes.
    - The threshold holds for this electrode placement and gain setting; it must be re-checked when either changes.
    - `tools/watch_myoware_uart.py` reads the new output format.

  * **Stage 3a: USART2 output, passes on the board (2026-08-15).** Questions 3 and 4 both need UART output, so UART is verified on its own first.
    - Register values checked in RM0368 (Rev 5) and the STM32F401 datasheet:
      - PA2/PA3 = AF7 (USART2 TX/RX), datasheet Table 9.
      - `RCC_AHB1ENR.GPIOAEN` (bit 0), `RCC_APB1ENR.USART2EN` (bit 17): RM0368 §6.3.9, §6.3.11.
      - `USART_CR1`, `USART_SR`, `USART_BRR` fields: RM0368 §19.6.
      - Baud rate (RM0368 §19.3.4, OVER8 = 0): 16 MHz HSI, 9600 baud → `BRR=0x0683` (0.02 % error). 9600 baud and 16× oversampling tolerate the uncalibrated HSI best.
      - Every register macro name was checked against the CMSIS header (`stm32f401xc.h`).
    - **`firmware/src/uart_hello_main.c`:** sends one line per second and blinks the LED, so a wiring fault can be told apart from a firmware fault.
    - **Result:** clean, repeated `"EdgeNeuro Stage 3a: UART alive"` at 9600 baud: register values, baud rate and wiring are right.

  * **Stage 3b: ADC1 on PA0, software-triggered, passes on the board.** Register values from RM0368 ch. 11. `firmware/src/adc_hello_main.c` prints the 12-bit value: 1279–1316 with a finger on the electrodes, far steadier than an open input.
  * **Stage 3c/3d: TIM2 TRGO triggers ADC1 at 1 kHz, passes on the board (2026-08-15). Question 3 answered.**
    - Register values (RM0368 ch. 13 and ch. 11): `TIM2EN`; `TIM2CLK = HCLK = 16 MHz` (APB1 prescaler 1); `MMS = 010` (update event as TRGO); `EXTEN = 01` (rising edge), `EXTSEL = 0110` (TIM2 TRGO).
    - `(PSC+1)×(ARR+1) = 16,000,000 / 1000 = 16,000`: `PSC=15`, `ARR=999`, an exact integer division.
    - **`firmware/src/timer_adc_1khz_main.c`:** the main loop polls `EOC`, counts samples and reports once per 1000 (9600 baud cannot carry every sample).
    - **Result:** reports arrive every 1.005 s, 1.005 s and 1.002 s, within the HSI's own tolerance: steady 1 kHz sampling.

  * **Stage 3e: real forearm, question 4 conditionally passed (2026-08-15):**
    - `timer_adc_1khz` reports each 1000-sample window's min and max; disposable electrodes on the muscle belly, near the wrist and at the elbow (reference):
      - **Relaxed** (7 windows): `min` 1114–1119, `max` 1211–1216.
      - **Clenched and held** (7 windows): `min` unchanged, `max` rising from 1212 to a steady **1227**.
      - The difference (11–16 counts, 0.3–0.4 % of full scale) is clean and repeatable: the chain from electrodes to ADC responds to real contractions, but the range is narrow.
    - The MyoWare 2.0 has a gain trim pot (`ENV gain = 200 × R / 1 kΩ`); the guide recommends a peak near VIN at full effort. The peak here (about 30 % of full scale) leaves room for more gain.
    - **Decision:** gain and electrode placement are tuned later; they do not block the remaining work. Question 4 is conditionally passed.

  * **Host-side work while waiting for hardware:** orientation fusion and command smoothing are plain math, fully testable on the host with Catch2:
    - **`include/edgeneuro/fusion/complementary_filter.hpp` (done, checked on real data):** `ComplementaryFilter<ValueType>` fuses gyroscope and accelerometer into roll and pitch (no yaw: there is no magnetometer). It is not a `Filter` (that concept is single-channel) and stays outside `Pipeline` (§3). `initialize(ax, ay, az)` starts from the accelerometer angle instead of converging from zero.
      - `tests/test_complementary_filter.cpp` (7 test cases) checks known physical inputs first (unit vectors, integrated known rates).
      - Units: the Myo protocol header (`myohw.h`) gives accelerometer in g, gyroscope in deg/s and a normalized quaternion; the data confirm it (|a| ≈ 1.03 g at rest, unit quaternions, single-digit gyro values).
      - `tests/test_complementary_filter_real_data.cpp` runs the filter on a still EMG-EPN-612 recording (992 rows) and compares with the Myo's own quaternion: 1.4e-4 rad (roll) and 2.8e-4 rad (pitch) apart, well within the 0.01 rad tolerance.
    - **`include/edgeneuro/control/grip_state_machine.hpp` (done, 2026-08-17):** `GripStateMachine<ValueType>` turns the MyoWare `ENV` signal into grip/release. The envelope must stay above `threshold` for `on_duration` to grip and below it for `off_duration` to release; those durations reject noise spikes. `update()` returns `true` only on a transition.
      - `tests/test_grip_state_machine.cpp` (7 test cases): start state, too short, exactly long enough, noise spikes, release, dips while gripping, `reset()`. Adding `0.01f` ten times gives slightly less than `0.1f`, so the tests use margins instead of exact step counts.
    - **`include/edgeneuro/control/slew_rate_limiter.hpp` (done, 2026-08-18):** `SlewRateLimiter<ValueType>` limits how fast the grip setpoint changes, turning the 0/1 decision into a ramp. It smooths a command, not a measurement. Residual IMU jitter can reuse `IirFilter`. Not a `Filter`: a rate needs `dt`.
      - `tests/test_slew_rate_limiter.cpp` (7 test cases): initial value, no overshoot, limited steps, ramps up and down, `reset()`, a moving target.
    - All 60 Catch2 test cases pass under `coverage` and `debug-heapguard`.

  * **`include/edgeneuro/control/threshold_calibrator.hpp` and an interactive boot calibration (2026-08-19):** instead of a fixed threshold, the firmware records a relaxed phase and a clenched phase and takes the midpoint between the relaxed maximum and the clenched minimum. `is_valid()` requires both phases to have been observed (the initial sentinel values alone satisfied `contracted_min > relaxed_max`; fixed and tested). `tests/test_threshold_calibrator.cpp` (7 test cases).
    - Fixed delays did not match when the person actually relaxed or clenched, so each phase now starts when a byte arrives on USART2. First clean result: `relaxed_max=491 contracted_min=3583 threshold=2037`.
    - Waiting for that byte can block boot indefinitely, so the calibration is off by default (`kCalibrationEnabled = false`, compiled out with `if constexpr`) and `kFallbackThreshold` keeps the last calibrated value, 2037.
    - **Electrode contact drifts:** within a few minutes of calibration, the relaxed signal rose close to the clenched level and oscillated around the threshold. Likely electrode contact or sweat; left for later.

  * **Stage 5b: EMG and IMU in one 1 kHz loop with non-blocking I2C (2026-08-20/21):** `firmware/src/phase3_control_loop_main.cpp` combines the Stage 5a EMG loop (`GripStateMachine` + `SlewRateLimiter`) with IMU fusion (`ComplementaryFilter`) in one main loop.
    - **Non-blocking I2C:** a 14-byte read at 100 kHz takes 1.5–2 ms, longer than the 1 ms EMG tick. `ImuReader` splits it into explicit states (`WaitStart1` … `WaitBtfLast`); `step()` checks one flag and returns. The sequence matches the verified blocking driver. Its timeout counts elapsed ticks, not calls.
    - With both MPU6050s faulty, the state machine was first tested against the LCD1602 (PCF8574, `0x27`), which goes through the same START/ADDR/TXE/BTF/RXNE/STOP sequence, including the multi-byte BTF tail. A real MPU6050 only needs `kImuTargetAddr` changed to `0x68`.
    - **Polling rate:** calling `step()` once per EMG tick gave only 47–48 reads/s, since a read needs ~22 state steps. Calling it on every loop pass, between ADC samples, raised that to **592–593 reads/s**, close to the bus limit. With steady operation `g_timeout_count` stays 0.
    - **`ComplementaryFilter::update()` takes `dt`:** reads now complete every 2–4 ticks, so a fixed `dt_` would integrate the gyroscope wrongly. A new overload takes the elapsed time; a test checks that it is used.
    - **Later:** the MPU6050 supports 400 kHz I2C, which could cut a read to about a quarter.
    - **IMU smoothing (2026-08-21):** `roll_smoother`/`pitch_smoother` reuse `IirFilter` as a one-pole EMA (`b0=alpha`, `a1=alpha-1`); `kSmoothAlpha=0.5` is a starting value, to be tuned on real jitter.
    - **`kSmoothAlpha` not yet tuned (2026-08-22):** at rest the real MPU6050 barely jitters after the complementary filter (α = 0.98), so its effect could not be seen; `kSmoothAlpha=0.5` stays unverified.
    - **Self-recovery:** a loose breadboard contact hung the bus again (`BUSY`, 758 timeouts in `WaitStart1`). A timeout now does a full `SWRST` and re-configuration, as at boot, so a temporary hang recovers on the next `begin()`.

  * **Stage 5c: a real Adafruit MPU-6050; fusion verified on the hardware (2026-08-22).** Four wires (`Vin` to 3.3 V, `GND`, `SCL` PB6, `SDA` PB7); address `0x68` with `AD0` open. `i2c_bus_scan` sees it at once.
    - **Bug found:** `phase3_control_loop_main.cpp` never woke the sensor (`PWR_MGMT_1`, 0x6B), so reads completed (591–593/s) but roll and pitch stayed 0. `mpu6050_write_reg_blocking()` now wakes it at boot.
    - **Result:** roll and pitch are steady at rest and follow tilting and turning in real time; the smoothed values follow with a small lag. Reads stay at 591–593/s, as measured with the LCD1602. Real IMU data → `ComplementaryFilter` → `IirFilter` works on the hardware.

  * **I2C fast mode (400 kHz): the chip and the math work, the breadboard does not (2026-08-22).** RM0368 18.6.8/18.6.9: `CCR=14`, `TRISE=6` at 16 MHz. **First test:** reads went from 592/s to **2028/s** (about 3.4×), with correct roll and pitch.
    - Shaking the sensor to look at jitter loosened the wiring: roll and pitch stuck at 0 with reads still completing, and the bus later hung at 100 kHz too. Re-seating the wires fixed both.
    - **Decisive test:** with the wiring working at 100 kHz and untouched, only switching `CCR`/`TRISE` back to 400 kHz reproduced the hang at once. The breadboard's capacitance likely pushes rise times past fast mode's 300 ns. **The firmware stays at 100 kHz;** the 400 kHz constants are kept for soldered wiring.

  * **Stage 6: live hardware drives MuJoCo; second MPU6050 and faster USART2. Code done, waiting for wiring (2026-08-22).** The CSV-replay bridge only checks the model and the mapping; the goal is the real STM32, MyoWare and two MPU6050s decoding live and driving MuJoCo.
    - **Two IMUs on one I2C1 bus:** upper arm `0x68` (`AD0` open), forearm `0x69` (`AD0` to 3.3 V). `ImuReader` and `mpu6050_write_reg_blocking()` take the address as a parameter; `shoulder_reader(0x68u)` and `elbow_reader(0x69u)` take turns (`active_is_shoulder`), so only one transfer runs at a time.
    - **Expected rate:** about 290–295 reads/s per sensor, half of Stage 5c's single-sensor rate; `shoulder_completions`/`elbow_completions` count them.
    - **USART2 at 115200 baud:** the new line (~95 bytes, matching `LINE_RE` in `tools/mujoco_bridge/run_demo.py`) needs more than 9600 baud. Nothing configures `RCC->CFGR`/`RCC->PLLCFGR`, so the clock is the 16 MHz HSI; RM0368 19.3.4 gives `USARTDIV=8.6875`, `BRR=0x8B` (115,108 baud, 0.08 % error). The line is sent every 10 ticks (100 Hz). `usart2_send_float()` prints decimals without `printf`.
    - **Fusion and elbow angle as in the CSV prototype:** two `ComplementaryFilter` instances, each initialized from its first reading; `elbow = max(0, elbow_filter.pitch() - shoulder_filter.pitch())`. The unverified `IirFilter` smoothing is removed, so the firmware matches the tested host prototype.
    - **`tools/mujoco_bridge/run_demo_live.py`:** reads USART2 with `pyserial` and reuses `run_demo.py`'s `LatestSample`, `LINE_RE`, actuator mapping and viewer loop; a separate script, so the CSV version stays untouched. Defaults to 115200 baud (`tools/watch_myoware_uart.py` uses 9600 for the earlier firmware). `pyserial` is in `requirements.txt`; `tools/check_hardware_ready.py` now opens the port at 115200.
    - **Verification plan, SWD first:** builds cleanly (`text=4104 data=536 bss=316`). Once the second MPU6050 is wired (`AD0`→3.3 V): ① `check_hardware_ready.py --i2c-scan` sees `0x68` and `0x69`; ② `g_wake_result_shoulder`/`g_wake_result_elbow` read over SWD; ③ `shoulder_completions`/`elbow_completions` near ~290/s; ④ the serial port opens; ⑤ raw lines in `miniterm` match the format; ⑥ moving each sensor moves the right joint in MuJoCo.

  * **Stage 6 on the hardware (2026-08-23):** dual IMU + MyoWare with `run_demo_live.py`. Several real hardware problems, each traced over SWD and the diagnostic counters:

    - **Unpowering IMU#2 also stopped IMU#1:** with only `VIN`/`GND` removed, an unpowered MPU6050 can hold `SDA` low and hang the shared bus. Per-reader `nack_count()` (this device stopped answering) and `timeout_count()` (the bus hung) now tell the two cases apart.
    - **Fix: `i2c1_bus_recovery()`, the bus clear of NXP UM10204 §3.1.16:** take `PB6`/`PB7` as GPIO, clock `SCL` up to 9 times until the slave releases `SDA`, send STOP, hand the pins back and `SWRST`. `g_bus_recovery_attempts`/`g_bus_recovery_freed` count it.
    - **Reads complete but the angles stay exactly `0.0`:** a sensor that lost power comes back with `SLEEP` set; it still answers, but its data registers never update. The boot-time wake-up does not cover a sensor that resets later.
    - **Fix:** any NACK or timeout marks the reader `needs_rewake_`, and its next completed read first re-sends the wake-up (`consume_needs_rewake()`). The trigger is the failure itself, not the look of the data, so recovery from a NACK and from a timeout are both covered.
    - **Known, deferred: power-on race.** The STM32 can try to wake a sensor before it has finished powering up, and then halts in `blink_code()`. Workaround: reflash with `check_hardware_ready.py --live-check`; the proposed fix (a short delay before the wake-up) is deferred.

  * **EMG stuck at a fixed value: a loose contact at the STM32 (2026-08-23).** Ruled out in turn: electrode placement, the MyoWare supply, I2C noise and the firmware (the earlier `adc_hello_main.c` showed the same). The MyoWare's own LED followed every clench, and a meter on the `ENV`→`PA0` wire responded, but `PA0` did not: the pin's breadboard contact had failed. Re-seated, the signal is clean (relaxed ~2000–2150, clenched ~3600–3700).
    - **Gain had drifted:** the relaxed level is far above the ~450 of 2026-08-18, so the MyoWare's gain pot had moved. The old `kFallbackThreshold=2037.0f` now sits inside the relaxed range; it is raised to `2800.0f`.

  * **Diagnostic tools no longer guess at disconnections (2026-08-23 → 25):**
    - **`tools/check_hardware_ready.py --live-check`** judged the forearm IMU by whether `elbow` changed, but `elbow` is a derived value clamped at 0 and can stay 0 in some poses. It now reads the firmware's `diag` line (`shoulder_completions`, `elbow_completions`, `nacks`, `timeouts`).
    - **`tools/mujoco_bridge/run_demo_live.py`** uses the same `diag` logic instead of watching values change. New: `--log-file`, a per-IMU retry count while offline (`imu_retry_attempts()`), and a multi-line `[SENSORS]` block.

  * **Hardware on hand (2026-08-29): five working MPU6050s** — three GY-521 and two Adafruit (`0x68` and `0x69`). A GY-521 drops into the upper-arm position (`0x68`) with no firmware change; the Adafruit boards stay the main ones (built-in pull-ups, better track record).

* **Phase 2: MuJoCo simulation — scheduled after Phase 1.5**
  * Bring in the **MuJoCo** physics simulator.
  * A bridge that maps the engine's output to a simulated hand (for example the Shadow Hand).
  * **Iteration 1: Python + MuJoCo bridge, done (2026-08-22).** The official MuJoCo Python package gives a working 3D demo first; a WebAssembly version can come later.
    - `src/mujoco_bridge_demo.cpp` replays `data/wearable_1emg_6imu.csv` through the hardware-verified `GripStateMachine`, `ComplementaryFilter` and `SlewRateLimiter` (not `Pipeline`, §3) at 1 kHz and prints `tick=<n> grip=<f> gripping=<0|1> roll=<f> pitch=<f>`, flushing each line so the pipe does not buffer.
    - The sensors match the verified hardware (one MPU-6050, one MyoWare 2.0), so MuJoCo shows what the real device would decode.
    - The hand is the Shadow Hand from `mujoco_menagerie` (Apache-2.0, `shadow_hand/scene_right.xml`, commit `da76818e269b82289eba39808e2fb91d679d6994`), fetched by sparse checkout and ignored by git.
    - `tools/mujoco_bridge/run_demo.py` starts the C++ binary, reads its output on a background thread, and applies the latest values in the `launch_passive`/`mj_step`/`viewer.sync()` loop: `grip` in [0,1] scales the finger actuators, `roll`/`pitch` drive the wrist (`rh_A_WRJ2`/`rh_A_WRJ1`). On macOS it needs `mjpython`.
    - **Result:** the fingers close and open with the EMG, the wrist follows the IMU data, and the viewer holds the last pose when the file ends.
    - A two-IMU arm was tried first and set aside because it did not match the hardware planned at the time; once two IMUs were confirmed the same day, it became Iteration 2.

  * **Iteration 2: two-IMU shoulder and elbow, with physical grasping; motion done, grasping a known limitation (2026-08-22).** The sensors match the planned hardware (2× MPU6050, 1× MyoWare).
    - **Sensors to joints:** upper-arm roll/pitch → shoulder (2 DOF); forearm pitch minus upper-arm pitch → elbow (1 DOF). Two `ComplementaryFilter` instances; output `tick=<n> grip=<f> gripping=<0|1> shoulder_pitch=<f> shoulder_roll=<f> elbow=<f>`. The wrist stays neutral: two IMUs cannot observe it separately.
    - **Arm model:** `tools/mujoco_bridge/arm_hand_scene.xml` copies the `rh_forearm` subtree from `right_hand.xml` unchanged and mounts it on a new shoulder (2 joints) and elbow (1 joint). MJCF `<include>` cannot re-parent a body, hence the copy; meshes still come from `mujoco_menagerie/`.
    - **Palm orientation fixed:** on the new arm the palm faced up when reaching; an extra 180° turn (`quat="1 0 1 0"`) points it down (normal from `(0.34,0,+0.94)` to `(-0.29,0,-0.96)`).
    - **Joint damping fixed:** the arm oscillated for hundreds of steps; damping and armature raised to 8/0.05 (shoulder) and 5/0.03 (elbow).
    - **Object:** a sphere instead of an ellipsoid; mass, friction and pedestal tuned against the simulation over time (see `arm_hand_scene.xml`).
    - **Known limitation:** even at full grip the fingertips stay 3–8 cm from the ball, which is carried for a while and then slips. Every finger closes by the same fraction; a real grasp needs per-finger targets (grasp synthesis), beyond this iteration. The arm motion itself works.

---

### Stage 2: Target-side embedded deployment and actuation (hardware)
Once Phase 1.5 answers its four questions, its results set how much goes into MuJoCo and how much into the full hardware build.

* **Phase 3: hardware abstraction (STM32 / sensor stream)**
  * CMake + `arm-none-eabi-gcc` (the `firmware/` project): ADC for the **MyoWare 2.0** and I2C for **two** MPU6050s (upper arm and forearm, §3), which needs a second reader instance.
  * A `Stm32AdcProvider` in place of `CsvSignalProvider` (timer-driven sampling).
  * The verified C++ core and ring buffer compile for the MCU unchanged.
  * **Hardware in the loop:** the **STM32F401RCT6 Black Pill** streams its decoded output over serial to the MuJoCo model on the PC.

* **Phase 4: closed-loop actuation**
  * Timer PWM drives servos.
  * The full loop: sensor sample $\rightarrow$ ring buffer $\rightarrow$ zero-allocation engine $\rightarrow$ PWM.
  * Practical issues: separate supplies for motors and sensors, supply noise, electrode contact.

---

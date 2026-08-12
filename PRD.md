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
| **Unit tests** | `Catch2` + `llvm-cov` | Over 90 % coverage of the core math and concept interfaces. **Measured:** 38 test cases over `include/edgeneuro/`; line 95.69 %, branch 90.32 % (re-measured 2026-08-12 with `bump_allocator.hpp`, 100 % covered). The uncovered lines are the `bad_alloc` path, an unused sized-delete overload, and the `NoHeapGuard` abort path, which `test_no_heap_guard.cpp` checks in a forked child. Measuring coverage found two real bugs: `CsvSignalProvider` silently truncated rows with extra columns, and the firmware's bump allocator returned unaligned memory and threw where no unwind tables exist. Both fixed and tested. |
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
    1. Does `NoHeapGuard` still see `malloc_count == 0` when the core is built with `arm-none-eabi-gcc` and run on the board?
    2. Does the pipeline fit in 64 KB SRAM / 256 KB Flash (`arm-none-eabi-size`)?
    3. Can a timer-triggered ADC sample steadily at 1 kHz?
    4. Is the MyoWare signal on a real forearm good enough for a reliable open/close decision?
  * Toolchain: plain CMake + `arm-none-eabi-gcc` in `firmware/`, no STM32CubeIDE. Diagnostics through the LED and UART.

  * **Hardware on hand (2026-08-11):** the Black Pill and one MyoWare 2.0. Ordered: an ST-Link V2 and a USB-serial adapter. Servos, MPU6050s and the arm come with Phase 3.

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

  * **Stages 0–2 compile (2026-08-12); not yet flashed:**
    - **Stage 0 (blink):** `text=212 bytes`. The toolchain compiles and links.
    - **Stage 1 (footprint):** the unmodified `include/edgeneuro/` headers build a real `EdgeNeuro<1,6,50,...>` for the STM32F401: `text=932 bytes, data=0, bss=0`, 0.38 % of the Flash.
    - **Stage 2 (`NoHeapGuard` on the target):** `no_heap_guard.hpp` is reused unchanged; `firmware/src/no_heap_guard_target.cpp` replaces `operator new`/`delete`, and a violation blinks the LED fast. `heap_guard_check_main.cpp` runs 100,000 `tick()` calls under the guard: LED solid = pass. `text=844 bytes, bss=8`.
    - **Flashing:** `firmware/openocd.cfg` (ST-Link, `stm32f4x`, 1 MHz). Every target gets a `flash_<name>` CMake target that programs it at `0x08004000`, leaving the bootloader in sector 0 untouched.

  * **Next:** flash `blink`, then `heap_guard_check` (LED solid = question 1 answered); then, with the USB-serial adapter, the ADC timing and the MyoWare signal (questions 3 and 4).

* **Phase 2: MuJoCo simulation — scheduled after Phase 1.5**
  * Bring in the **MuJoCo** physics simulator.
  * A bridge that maps the engine's output to a simulated hand (for example the Shadow Hand).

---

### Stage 2: Target-side embedded deployment and actuation (hardware)
Once Phase 1.5 answers its four questions, its results set how much goes into MuJoCo and how much into the full hardware build.

* **Phase 3: hardware abstraction (STM32 / sensor stream)**
  * CMake + `arm-none-eabi-gcc` (the `firmware/` project): ADC for the **MyoWare 2.0** and I2C for the MPU6050.
  * A `Stm32AdcProvider` in place of `CsvSignalProvider` (timer-driven sampling).
  * The verified C++ core and ring buffer compile for the MCU unchanged.
  * **Hardware in the loop:** the **STM32F401RCT6 Black Pill** streams its decoded output over serial to the MuJoCo model on the PC.

* **Phase 4: closed-loop actuation**
  * Timer PWM drives servos.
  * The full loop: sensor sample $\rightarrow$ ring buffer $\rightarrow$ zero-allocation engine $\rightarrow$ PWM.
  * Practical issues: separate supplies for motors and sensors, supply noise, electrode contact.
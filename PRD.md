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
| **Unit tests** | `Catch2` + `llvm-cov` | Over 90 % coverage of the core math and concept interfaces. **Measured:** 31 test cases over `include/edgeneuro/`; line 95.38 %, branch 90.00 %. The uncovered lines are the `bad_alloc` path, an unused sized-delete overload, and the `NoHeapGuard` abort path, which `test_no_heap_guard.cpp` checks in a forked child. Measuring coverage found a real bug: `CsvSignalProvider` silently truncated rows with extra columns; fixed and tested. |
| **No allocation, no undefined behaviour** | Sanitizers + `NoHeapGuard` | ASan, UBSan and TSan, plus a test that `malloc_count == 0` in the loop. ASan and TSan cannot share a binary and the allocation hook conflicts with them, so each runs from its own CMake preset (`debug-heapguard`, `sanitize-asan-ubsan`, `sanitize-tsan`, `release-bench`). |
| **Latency** | `Google Benchmark` | `<1, 6>` and `<32, 0>`: under 0.1 ms per sample with 32 channels, `malloc_count == 0`. |

## 6. Roadmap

The engine is built and verified on the PC first; MuJoCo simulation and the move to embedded hardware follow once it passes its tests.

```
+-----------------------------------------------------------------------------------+
| Phase 1: engine on the PC                                                        |
| C++20 concepts, lock-free ring buffer, zero allocation                           |
+-----------------------------------------------------------------------------------+
                                        |
                         tests pass before moving on
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

* **Phase 1: engine (C++20, PC) — in progress**
  * C++20 concepts for `SignalProvider`, `Filter`, `Feature`, `Classifier`, and a `ValueType` parameter.
  * A lock-free ring buffer on `std::array`.
  * `CsvSignalProvider` streams recorded data as if it came from a sensor.
  * `Catch2` tests; ASan, UBSan, TSan and `NoHeapGuard` (`malloc_count == 0`); `Google Benchmark` for latency under 1 ms.
  * A terminal view (`src/main.cpp`) and an ImGui + ImPlot window (`src/gui_demo.cpp`) show the signals and the allocation count.

* **Phase 2: MuJoCo simulation — after Phase 1**
  * Bring in the **MuJoCo** physics simulator.
  * A bridge that maps the engine's output to a simulated hand (for example the Shadow Hand).

---

### Stage 2: Target-side embedded deployment and actuation (hardware)
After the engine and the simulation are verified, the firmware moves onto a microcontroller with real sensors.

* **Phase 3: hardware abstraction (STM32 / sensor stream)**
  * ARM GCC toolchain; ADC for the MyoWare EMG and I2C for the MPU6050.
  * A `Stm32AdcProvider` in place of `CsvSignalProvider` (timer-driven sampling).
  * The verified C++ core and ring buffer compile for the MCU unchanged.
  * **Hardware in the loop:** the STM32 streams its decoded output over serial to the MuJoCo model on the PC.

* **Phase 4: closed-loop actuation**
  * Timer PWM drives servos.
  * The full loop: sensor sample $\rightarrow$ ring buffer $\rightarrow$ zero-allocation engine $\rightarrow$ PWM.
  * Practical issues: separate supplies for motors and sensors, supply noise, electrode contact.
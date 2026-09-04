# Development log

Key milestones and findings, oldest first. Each entry is one decision or one lesson; the numbers come from raw data
captured on the real hardware. For how the system works now, see [README.md](README.md) and [PRD.md](PRD.md). Code
comments that cite "SESSION_LOG <date>" refer to the dated entries below.

## August 2026: engine and hardware bring-up

- **08-04 → 08-12: zero-allocation C++20 engine.** Concept-checked pipeline (IIR, MAV/RMS, LDA, lock-free ring
  buffer), with `malloc_count == 0` enforced by a heap guard. Measuring coverage exposed two real bugs: silent CSV-row
  truncation and an unaligned allocator. Runs on the public EMG-EPN-612 dataset.
- **08-14: on the STM32.** 100,000 guarded ticks on the target with no allocation. Apps are linked after the 16 KB
  bootloader and flashed over ST-Link.
- **08-15: one peripheral at a time.** UART, ADC, then a timer-triggered ADC: exactly 1 kHz (1.002–1.005 s per 1000
  samples). Multi-byte I2C reads only worked once the `BTF` sequence from RM0368 §18.3.3 was followed.
- **08-17 → 08-18: faulty sensors isolated, EMG grip works.** A bus scanner plus a known-good I2C device proved two
  IMU modules faulty, not the firmware. The EMG grip separated relaxed (~450) from clenched (~3700).
- **08-20 → 08-22: a loop that never blocks.** I2C reads became a state machine; polling it on every loop pass instead
  of once per tick raised reads from 47/s to 592/s. Second IMU on the same bus at `0x69`.
- **08-23: hot-swap safety.** A standard I2C bus clear (UM10204) and an automatic re-wake recover a sensor that drops
  out, instead of the whole bus hanging.

## September 2026: live control, then the physical arm

- **09-01 → 09-05: angles that hold at any pose.** Elbow = angle between the two gravity vectors; shoulder =
  tilt/azimuth. Both replace Euler angles, which fold back past ±90°. Calibration from four natural poses. Rule set: judge
  an algorithm only by raw sensor data, never by its own output.

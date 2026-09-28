# Development log

Key milestones and findings, oldest first. Each entry is one decision or one lesson; the numbers come from raw data
captured on the real hardware. For how the system works now, see [README.md](README.md) and [PRD.md](PRD.md). Code
comments that cite "SESSION_LOG <date>" refer to the dated entries below; "TODO C" was the measurement of the arm's
shoulder × elbow region (09-24 → 09-27).

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
- **09-08 → 09-11: calibration moves to the PC.** Thresholds are sent live over UART, with no reflashing and no boot
  gate. UART dropouts were traced to three separate hardware causes, each fixed.
- **09-12 → 09-13: tremor filtered, first full task (v1.0.0).** Gripping caused 9–10 Hz physiological tremor. The
  IMUs' 5 Hz low-pass filter cut it 26–90×. Reach → grip → lift → hold then ran end to end on live hardware.
- **09-21 → 09-23: the MeArm.** Direct joint mapping instead of inverse kinematics. "Dead" servos were the supply
  sagging during current spikes, fixed with a bulk capacitor.
- **09-24 → 09-27: safe by measurement.** MuJoCo twin; Python reference mapping with C++ ports tested against golden
  tables. The arm's coupled shoulder × elbow limits were measured in five runs and frozen as a conservative envelope.
- **09-27 → 09-28: a wiring fault, not a bug (v1.1.0).** A day of "reversed" motion was a loose IMU: rebuilding v1.0.0
  showed the same symptom. Since then, sensor faults are detected automatically and the arm holds still.

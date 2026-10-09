# Development log

Key milestones and findings, oldest first. Each entry is one decision or one lesson; the numbers come from raw data
captured on the real hardware. For how the system works now, see [README.md](README.md) and [PRD.md](PRD.md). Code
comments that cite "SESSION_LOG <date>" refer to the dated entries below; "TODO C" was the measurement of the arm's
shoulder × elbow region (09-26 → 09-28).

## August 2026: engine and hardware bring-up

- **08-04 → 08-12: zero-allocation C++20 engine.** Concept-checked pipeline (IIR, MAV/RMS, LDA) and a lock-free
  ring buffer, with `malloc_count == 0` enforced by a heap guard. Measuring coverage exposed two real bugs: silent CSV-row
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

- **09-01 → 09-05: angles that hold at any pose.** Elbow = angle between the two gravity vectors; shoulder = the
  upper-arm vector decomposed along calibrated FORWARD and LEFT_TWIST directions. Both replace Euler angles, which fold
  back past ±90°. Calibration from four natural poses. Rule set: judge
  an algorithm only by raw sensor data, never by its own output.
- **09-08 → 09-11: calibration moves to the PC.** Thresholds are sent live over UART, with no reflashing and no boot
  gate. UART dropouts were traced to three separate hardware causes: a dead TX pin on the first board
  (board replaced), a known CP2102 lockup (adapter replaced with an FT232RL), and BOOT0 occasionally read high (detected
  over SWD, not fixed).
- **09-12 → 09-13: tremor filtered, first full task (v1.0.0).** Gripping caused 9–10 Hz physiological tremor. The
  IMUs' 5 Hz low-pass filter cut it 26–90×. Reach → grip → lift → hold then ran end to end on live hardware.
- **09-21 → 09-23: the MeArm.** Direct joint mapping instead of inverse kinematics. "Dead" servos were the supply
  sagging during current spikes, fixed with a bulk capacitor.
- **09-24 → 09-28: safe by measurement.** A MuJoCo MeArm model; Python reference mapping with C++ ports tested against golden
  tables. The arm's coupled shoulder × elbow limits were measured in five runs and frozen as a conservative envelope.
- **09-27 → 09-28: a wiring fault, not a bug (v1.1.0).** A day of "reversed" motion was a loose IMU: rebuilding v1.0.0
  showed the same symptom. Since then, sensor faults are detected automatically and the arm holds still.
- **09-29 → 10-01: servo power.** Dedicated battery pack, bulk capacitors, common ground: most servo stalls disappeared.

## October 2026: the real arm follows the person

- **10-03: timing, start-up and steadiness.**
  - A UART busy-wait had cut the 1 kHz loop to 291 ticks/s; a transmit queue restored 1007.
  - The FPU had never been enabled at reset and only worked because the bootloader left it on. Fixed.
  - The base jitter was the arm's own sway, so the servos stay at full speed and the command is filtered (1€ filter +
    hysteresis).
  - Height/reach mapping, calibration sent over UART, and an `R` command to home the arm.
- **10-04: reliable links and a steady base (v1.2.0), then a two-threshold grip (released in v1.3.0).** The calibration
  message was losing bytes; it is now paced and resent, and 5 of 5 sends were applied. The base follows slowly while the
  arm is raised, cutting unwanted swing 3–5×. Release threshold set below the grip threshold. Recordings showed the remaining grip problem:
  arm motion alone (3848) out-reads a firm grip (1318). That is electrode placement, which no threshold can fix.
- **10-07: robust wiring (v1.3.0).** Breadboard replaced by lever connectors: bus errors fell from ~15/s to 1 in 28 s
  of motion. The two-threshold grip was verified with the demo motions.
- **10-08: latency on the target.** At 16 MHz the real configuration takes 15 µs per sample (1.5 % CPU). A 32-channel
  stress test overruns 1 ms on classify ticks, so it would need the faster clock.
- **10-09: the full task on the real arm.** Hang → forward → swing left → grip a roll of tape → lift → swing right →
  place, completed end to end on the physical MeArm, controlled live by the person wearing the sensors (recorded on
  video). One tuned task, one user; a success rate over repeated trials is still to be measured.

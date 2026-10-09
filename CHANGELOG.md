# Changelog

All notable changes to this project. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versions follow [Semantic Versioning](https://semver.org/). What was verified on the real hardware (STM32F401 Black Pill,
2× MPU6050, MyoWare 2.0, MeArm) for each release is in its GitHub Release notes; the full story behind each item is in
[`SESSION_LOG.md`](SESSION_LOG.md).

## [Unreleased]

### Added
- **The full demo task completed on the physical arm** (2026-10-09, on video): hang → forward → swing left → grip a
  roll of tape → lift → swing right → place, controlled live by one person wearing the sensors.
- **Pipeline latency measured on the STM32F401** ([#13]): the host benchmark's two configurations now live in a shared
  header, and the new `pipeline_latency` firmware times them with the DWT cycle counter. At 16 MHz: `<1,6>` 15 µs per
  sample on average (317 µs on the classify tick); `<32,0>` 134 µs on average, but its 1.54 ms classify tick overruns a
  1 ms sample period.

### Changed
- Documentation rewritten in English to match the current code: README, firmware README, PRD (now a design and
  requirements document) and a condensed development log. Stale references in code comments were updated.

## [1.3.0] — 2026-10-08

A stable point for the MeArm demo task (hang → forward → left → grip a tape roll → lift → right → place): the EMG grip
holds while the arm moves, and the hardware is documented as it is now wired. The user verified this code on the real
arm on 2026-10-07.

### Changed
- **EMG grip uses two thresholds (hysteresis)** ([#6]). The grip still needs the calibrated threshold, but letting go
  needs falling below a lower release threshold, halfway between the relaxed level and the grip threshold. A gentler
  grip held while the arm moves no longer lets go.
  - The UART `T` command accepts `T<on>,<release>` as well as the old `T<on>`. A malformed line is dropped whole.
  - The calibration computes, sends and saves the release threshold. `--skip-emg-calibration` sends a saved one.

### Added
- **EMG recorded alongside arm motion** ([#7]): `capture_arm_motion.py --set emg` records relaxed and gripping, still
  and during the demo motion, and replays the board's grip decision to count false grips and false releases.
- **Black Pill pin map and wiring notes** ([#8]) in the README: every pin and what it connects to, the sensor-side WAGO
  wiring, and what a blinking LED at boot means.
- **Recorded data and reference photo** ([#9]): the EMG motion recording behind the analysis in `SESSION_LOG.md`, and
  the photo that shows which MeArm link to measure.

### Fixed
- Sensor wiring moved from a breadboard to WAGO lever connectors. With the loose contact found by a wiggle test, the
  upper-arm IMU went from about 15 dropouts a second (2026-09-28) to 1 in 28 s of arm movement (hardware change,
  documented in [#8]).

## [1.2.0] — 2026-10-04

The physical MeArm follows the user's arm, computed on the STM32 itself from the two IMUs and the EMG sensor ([#5]).

### Added
- Base driven by upper-arm twist; shoulder and elbow set the claw's height and reach (`height_reach`), always inside the
  measured linkage envelope, including intermediate poses.
- Claw follows the EMG grip. `--mearm` sends the calibrated EMG threshold.
- 1-euro filter on both IMUs, hysteresis on the base, and slow base follow while the arm is being raised.
- Calibration sent to the board over UART at startup (checksummed, validated, resent on a lost byte), so recalibrating
  needs no reflash.
- `R` command: every servo walks back to the start pose.
- Tools for measuring the linkage envelope, servo angles and servo response, and an interactive arm-motion capture.

### Fixed
- The FPU was never enabled in `Reset_Handler` (the apps only ran because the bootloader left it on). The SWD direct
  jump now starts the app reliably.
- UART output is queued instead of busy-waiting, which had slowed the 1 kHz main loop 3.4×.

## [1.1.0] — 2026-09-28

Automatic IMU health checks, so a hardware fault cannot pass for a wrong model ([#4]).

### Added
- `run_demo_live.py` refuses to start on faulty IMU data (no data, frozen, stuck at full scale, |a| not ≈ 1 g, asleep),
  holds the arm on a fault mid-session, and names a flaky-but-recovering sensor.
- The firmware reads each MPU6050's `PWR_MGMT_1` once a second and wakes a sensor that reset.
- `check_hardware_ready.py --sensors`; `--i2c-scan` requires both 0x68 and 0x69; `tools/watch_imu_raw.py`.
- Golden calibration `data/shoulder_calibration_golden_2026-09-13.json`.
- Servo PWM bring-up firmware for the MeArm's four servos and real arm-range logging ([#2]).
- `CLAUDE.md`: working conventions for this repo ([#3]).

## [1.0.0] — 2026-09-13

First stable release: two MPU6050s and the MyoWare EMG drive a full reach → grip → lift → hold cycle on real hardware
end to end, in the MuJoCo humanoid arm.

### Added
- Zero-heap-allocation C++ control loop on the STM32F401 (1 kHz EMG, non-blocking I2C for both IMUs).
- Live bridge into MuJoCo (`run_demo_live.py`) with pose and EMG calibration.
- DLPF and EMA tremor smoothing; EMG threshold as mean + K·std, with a long-term log of real calibration sessions.
- Sensors can be marked optional instead of crashing when absent ([#1]); a firmware compile job in CI.

[Unreleased]: https://github.com/a8901147/neuroEdge/compare/v1.3.0...HEAD
[1.3.0]: https://github.com/a8901147/neuroEdge/compare/v1.2.0...v1.3.0
[1.2.0]: https://github.com/a8901147/neuroEdge/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/a8901147/neuroEdge/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/a8901147/neuroEdge/releases/tag/v1.0.0
[#1]: https://github.com/a8901147/neuroEdge/pull/1
[#2]: https://github.com/a8901147/neuroEdge/pull/2
[#3]: https://github.com/a8901147/neuroEdge/pull/3
[#4]: https://github.com/a8901147/neuroEdge/pull/4
[#5]: https://github.com/a8901147/neuroEdge/pull/5
[#6]: https://github.com/a8901147/neuroEdge/pull/6
[#7]: https://github.com/a8901147/neuroEdge/pull/7
[#8]: https://github.com/a8901147/neuroEdge/pull/8
[#9]: https://github.com/a8901147/neuroEdge/pull/9
[#13]: https://github.com/a8901147/neuroEdge/pull/13

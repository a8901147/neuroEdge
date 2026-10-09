# EdgeNeuro: Design and Requirements

**Author:** Chang-Jui Tseng · **Last updated:** 2026-10-09

This document covers what the system is for, how it is built, the requirements it has to meet, and how each one is
checked. Commands are in [README.md](README.md) and [firmware/README.md](firmware/README.md); the dated record of how
the design got here is in [SESSION_LOG.md](SESSION_LOG.md).

## 1. Problem and goal

Myoelectric prosthetic arms are costly, and many users abandon them (Biddiss & Chau, 2007). Part of the reason is
control: users switch modes to move one joint at a time, grips drop or trigger when the arm moves, and multi-gesture
pattern recognition needs many EMG channels and per-user training. Delay matters too: Farrell & Weir (2007) found
100–125 ms to be the best controller delay for myoelectric control.

**Goal:** natural, simultaneous control of an arm (direction, height, reach and grip) from inexpensive wearable sensors
(one EMG channel, two IMUs), with every computation on a low-cost microcontroller, smooth enough and reliable enough
for a real pick-and-place task. The benchmark task, on a 4-servo MeArm:

> arm hanging → reach forward → swing left → grip a roll of tape → lift → swing right → place.

Everything is tuned toward this task without being specific to it: the code contains no task-specific poses, and the
calibration and parameters come from the person's own recordings. **Status** (one run on video, 2026-10-09): the arm hooked the
tape roll through its core, lifted it and carried it to the right, and released it there (EMG relax), so it dropped
from claw height; a lowered set-down and a success rate over repeated trials are still to be shown.

**Scope:** a research platform. An able-bodied person controls a desktop arm and MuJoCo models; the system has not
been tested with prosthesis users.

## 2. Approach

- **Run the control on the microcontroller.** The STM32 computes the servo commands itself; the PC calibrates, sends the
  home command and visualizes.
- **Deterministic, allocation-free C++.** Header-only C++20 with templates and concepts, so there are no virtual calls
  and no heap use in the hot loop. The same headers compile for the host (where they are tested) and for the
  Cortex-M4F.
- **Validate in simulation and on hardware together.** The live sensor stream also drives two MuJoCo models (a MeArm
  model and a Unitree G1 humanoid arm and hand), computed on the PC. They show what the sensors decode to; they are not
  a twin of the physical arm, whose servo mapping, filtering and limits run on the STM32 (§5).
- **Raw data decides.** Thresholds and filter parameters are settled with captured raw sensor data, never with an
  algorithm's own output; register values are checked against the reference manual (RM0368).

## 3. System architecture

```
Sensors                       STM32F401 (1 kHz loop; servo commands every 10th tick = 100 Hz; zero heap)   Outputs
MyoWare ENV ─ADC─▶ EMA ─▶ GripStateMachine (2 thresholds, 150 ms debounce) ─▶ slew limit ─▶ claw servo
MPU6050 ×2  ─I2C─▶ raw gravity vectors ─▶ 1€ filter ─▶ Path B decode (tilt, azimuth)
                                                     ─▶ height/reach mapping ─▶ envelope ─▶ shoulder, elbow servos
                                                     ─▶ base mapping, hysteresis, slow follow ─▶ base servo
                   health checks ─▶ hold all servos on a sensor fault
                   UART ◀▶ PC: raw stream out; calibration, thresholds, R (home) in
PC (Python)        calibration · MuJoCo MeArm model · MuJoCo Unitree G1 arm+hand (computed on the PC)
```

The library has two layers:

1. **A generic pipeline, `EdgeNeuro<ValueType, EmgChannels, ImuChannels, WindowSize, Provider, EmgFilterT, ImuFilterT,
   FeatureT, ClassifierT>`** (`include/edgeneuro/pipeline.hpp`). It is assembled at compile time from strategies
   checked by C++20 concepts: IIR and pass-through filters, MAV/RMS features, an LDA classifier and a CSV provider. It
   shows that the DSP stays modular at zero runtime cost and with zero allocation, and it is what the latency
   benchmarks measure (`<1,6>` and `<32,0>`). A lock-free SPSC ring buffer (`ring_buffer.hpp`) is provided and
   ThreadSanitizer-tested as a separate component; neither the pipeline nor the firmware uses it at present.
2. **The real control path** (`include/edgeneuro/control/`, `filters/`, `fusion/`). The hardware has one EMG channel,
   and one channel only carries "how hard is this muscle working". A window-and-classify step would add a window's
   worth of latency and gain nothing, so the grip is a threshold state machine and the arm follows the IMUs
   continuously. Neither goes through the windowed pipeline.

## 4. Hardware

| Part | Choice | Why |
| --- | --- | --- |
| MCU | STM32F401RCT6 Black Pill, 16 MHz HSI | Cortex-M4 with a single-precision FPU (matches `float`), 64 KB SRAM, low cost |
| EMG | MyoWare 2.0, `ENV` output to ADC | Analog front end in hardware (MyoWare 2.0 Advanced Guide): amplifier, first-order 20.8 Hz high-pass, full-wave rectifier, 3.6 Hz envelope |
| IMUs | 2× MPU6050 (upper arm 0x68, forearm 0x69), DLPF 5 Hz | Gravity direction of each arm segment: control uses only the accelerometers; the gyroscopes are read but unused. No magnetometer, so no heading |
| Arm | MeArm, 4× SG92R servos, 4×AA supply | 4 DOF: base, shoulder, elbow, claw |
| Links | ST-Link (SWD), FT232RL (UART 115200) | SWD register reads are the main debugging tool |

## 5. Control design

**Grip (EMG).** The 12-bit ENV sample is smoothed (EMA, α = 0.1, ~10 ms) and fed to a two-threshold state machine:
the grip closes after 150 ms above the grip threshold and opens after 150 ms below a lower release threshold. The PC
records 3.5 s relaxed and 3.5 s gripping and uses the last 2 s of each: the grip threshold is the relaxed mean + 20·std, and the release threshold is
halfway between the relaxed mean and the grip threshold. The hysteresis lets a lighter grip hold while the arm moves.

**Arm direction ("Path B").** The upper-arm accelerometer gives a gravity vector. Four calibration poses (hang,
forward, left twist, right twist) define a frame, and the vector is decoded into tilt (how far the arm is raised) and
azimuth (left/right, carried by upper-arm twist, because an accelerometer can't see rotation about gravity). The
elbow bend is the angle between the two IMUs' gravity vectors, which has no singularity anywhere in 0–180°.

**MeArm mapping ("height/reach").** On the MeArm, the forearm servo sets the claw's height and the upper-arm servo
sets its reach, so the person's arm maps crosswise onto it: raising the arm lowers the elbow servo (claw up), and
bending the elbow raises the shoulder servo (reach). Every (shoulder, elbow) command is clamped into an envelope
measured on the real arm (five measurement runs; between measured shoulder positions the windows are intersected,
never interpolated; the shoulder is clamped to the measured range). The two servos also step
together, so the poses in between stay inside the envelope too. Base: azimuth over the full 500–2500 µs, with the ends
at the person's comfortable left and right reach.

**Sensor-to-DOF mapping.** Three sensors give three inputs: the upper-arm gravity vector, the elbow bend (angle
between the two gravity vectors) and the EMG grip. What each target drives, from the code
(`include/edgeneuro/control/mearm_real.hpp` / `mearm_drive.hpp` for the physical MeArm, `tools/mujoco_bridge/run_demo_live.py`
and `arm_hand_scene.xml` for the G1). The MuJoCo MeArm model is driven differently from the physical arm: joint to
joint through `mearm_pathb.ctrl_from_sensors` (tilt → model shoulder, elbow bend → model elbow), without the
height/reach crossover, the 1€ filter, the base slow-follow or the measured envelope.

| Input | Physical MeArm (4 servos = 4 DOF) | Unitree G1 model, left arm + hand (14 actuators) |
| --- | --- | --- |
| Upper-arm gravity vector | decoded to tilt and azimuth (Path B): azimuth → base, tilt → elbow servo (claw height) | decomposed along the calibrated FORWARD and LEFT_TWIST directions → shoulder pitch and shoulder roll |
| Elbow bend | shoulder servo (reach) | elbow |
| EMG grip | claw | 6 finger joints together (uniform curl) |
| Not driven | — | shoulder yaw and wrist roll/pitch/yaw (no sensor measures them), thumb opposition (not a curl joint); all held at 0 |

**Steadiness.** The servos run at full speed, and the command is smoothed instead: DLPF 5 Hz on the sensors (this
suppresses 8–12 Hz physiological tremor while gripping), a 1€ filter on the raw vectors (0.5 Hz at rest, opening up
with speed), 10 µs hysteresis on the base, and a slow-follow rule (150 µs/s) on the base while the arm is being
raised, where the azimuth is unreliable near hanging.

**Safety.** The servo outputs are compiled out by default. Each servo starts with a slow ramp (300 µs/s). The `R`
command homes the arm to a known start pose. On the board (`include/edgeneuro/control/imu_health.hpp`), an IMU
reading that is implausible (an axis at full scale, or a magnitude outside 0.3–3 g) or unchanged for 0.3 s makes every
servo hold its pulse; a sensor found reset (`PWR_MGMT_1`) is woken again. On the PC (`tools/sensor_health.py`), missing
data, frozen readings and a magnitude far from 1 g are caught as well, and the model holds its pose. A calibration
received over UART is checksummed and validated before use.

## 6. Requirements and validation

| # | Requirement | Validated by | Status |
| --- | --- | --- | --- |
| R1 | No heap allocation in the hot loop | `NoHeapGuard` (global `operator new` hook) armed around `tick()` in host tests, and on the target in `heap_guard_check`; the firmware image links no `malloc`/`free`/`_sbrk` | Met |
| R2 | Fixed 1 kHz EMG sampling | TIM2 TRGO → ADC1; report intervals 1.002–1.005 s; 1007 ticks/s in the full loop | Met |
| R3 | IMU reads never block a tick | Non-blocking I2C state machine; ~274 reads/s per IMU with both on one bus | Met |
| R4 | No undefined behavior; ring buffer correct under concurrency | ASan + UBSan and TSan presets | Met (all three pass, 2026-10-08; CI runs ASan/UBSan) |
| R5 | Code tested | 248 Catch2 cases (98.7 % line / 88.6 % branch); 569 Python test functions; C++ ports checked against Python golden tables; mutation testing on new logic | Met |
| R6 | Fits the MCU | Servo build: 18.4 KB code, 3.3 KB static RAM | Met |
| R7 | The generic `EdgeNeuro<>` pipeline fits a 1 ms sample period on target (the arm's control path does not use it; its timing is R2) | DWT cycle counts at 16 MHz: `<1,6>` mean 15 µs, classify tick 317 µs | Met for `<1,6>`; `<32,0>` classify tick 1.54 ms overruns |
| R8 | Arm stays inside its mechanical limits | Envelope measured on the arm; host tests over the intermediate poses; recorded CCR traces | Met |
| R9 | An IMU fault never moves the arm | `ImuHealth` in firmware, `sensor_health` on the PC; host tests | Met for the IMUs; the EMG channel is not health-checked (§7) |
| R10 | Grip is reliable during arm motion | Two-threshold grip verified with demo motions on 2026-10-07; tape picked up, lifted and carried to the right on the real arm, 2026-10-09 (video; released at claw height rather than set down); motion artifact measured | Partly: limited by electrode placement (§7); the claw hooks the roll rather than clamping it; no controlled set-down or repeated-trial success rate yet |

## 7. Known limitations

- **EMG motion artifact.** With the current electrode placement, arm motion alone reached 3848 ADC counts, more than
  a firm still grip (1318). The MyoWare's high-pass corner (20.8 Hz) is near the 20 Hz De Luca et al. recommend against
  movement artifact, though it is first-order rather than their 12 dB/octave; gating on arm motion only cut false grips
  from 4 to 2. The fix is electrode placement over the finger flexors.
- **No heading.** Without a magnetometer, left/right comes from upper-arm twist. The twist that naturally comes with
  raising the arm is reduced by the slow-follow rule, not removed.
- **Clock.** At the default 16 MHz, the 32-channel stress configuration overruns 1 ms on classify ticks. Configuring
  the PLL for 84 MHz would give up to about 5× headroom.
- **Humanoid grasp.** The G1 hand's uniform-curl grasp holds the object only at the front-left pose. Per-finger grasp
  synthesis was rejected as out of scope.
- **EMG not health-checked.** Only the IMUs are checked; a loose electrode can open or close the claw.
- **No end-to-end latency measurement yet** (arm motion → servo motion); see §8.
- **UART receive** is polled one byte per loop, so the host paces commands (2 ms per byte).

## 8. Next steps

- Complete the set-down step (lower, then release) and measure the demo's success rate over repeated trials (and with a
  second person).
- Move the EMG electrodes to the finger flexors and re-record with `capture_arm_motion.py --set emg`.
- Measure end-to-end latency (sensor motion → servo motion) with high-speed video, and compare the grip's 150 ms
  debounce against the 100–125 ms that Farrell & Weir (2007) found best for myoelectric control.
- Raise the clock to 84 MHz (PLL), with the register values verified against RM0368.
- RXNE-interrupt UART receive; 400 kHz I2C on soldered wiring.

**Considered and dropped:** a browser (Wasm) demo, a HAL-based `Stm32AdcProvider` with DMA (the polled TIM2/ADC path
meets every requirement), per-finger grasp control, and a gyro-based base (gyro noise while gripping was ~100× the
noise at rest).

## 9. References

- Biddiss, E. & Chau, T. (2007). Upper limb prosthesis use and abandonment: a survey of the last 25 years.
  *Prosthetics and Orthotics International*, 31(3), 236–257.
- Farrell, T. R. & Weir, R. F. (2007). The optimal controller delay for myoelectric prostheses. *IEEE Transactions on
  Neural Systems and Rehabilitation Engineering*, 15(1), 111–118.
- Advancer Technologies / SparkFun. *MyoWare 2.0 Muscle Sensor: Advanced Guide* (filter specifications).
- De Luca, C. J. et al. (2010). Filtering the surface EMG signal: movement artifact and baseline noise contamination.
  *Journal of Biomechanics*, 43, 1573–1579.

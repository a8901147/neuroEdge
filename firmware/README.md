# firmware/

Bare-metal firmware for the STM32F401RCT6 Black Pill: plain CMake + `arm-none-eabi-gcc`, CMSIS register headers, a
hand-written vector table and start-up code, with no HAL and no RTOS. The rule in this repo is that a register field is
checked against RM0368 (or the vendored CMSIS header) before it is written, and the source comments cite the section.

The product firmware is `phase3_control_loop`. The other targets are bring-up steps, each of which proved one
peripheral or one claim on the real board before it was combined into the main loop, plus diagnostic and measurement
targets.

## Hardware

| Part | Notes |
| --- | --- |
| MCU | STM32F401RCT6 Black Pill (Cortex-M4F, 256 KB Flash, 64 KB SRAM). Runs at the reset-default 16 MHz HSI; no PLL is configured. |
| Bootloader | WeAct HID bootloader in the first 16 KB of Flash. Apps are linked at `0x08004000` (`linker/STM32F401RCTx_FLASH.ld`); don't move it back to `0x08000000`, which would erase the bootloader. |
| EMG | MyoWare 2.0, `ENV` (rectified, low-passed envelope) output |
| IMUs | 2× MPU6050 breakout boards on one I2C bus: upper arm `0x68` (AD0 open), forearm `0x69` (AD0 to 3.3 V) |
| Arm | MeArm with 4× SG92R servos, on a separate 4×AA supply |
| Debug | ST-Link V2 (SWD) for flashing and register reads; FT232RL USB-serial adapter for UART (avoid CP2102: see `tools/usb_serial_port.py`) |

### Pin map

| Pin | Function | Connects to |
| --- | --- | --- |
| PA0 | ADC1_IN0 | MyoWare `ENV` |
| PA2 / PA3 | USART2 TX / RX, 115200 8N1 | USB-serial `RXD` / `TXD` (crossed), plus `GND`; its `VCC` is not connected |
| PA6 / PA7 / PB0 / PB1 | TIM3 CH1–CH4, 50 Hz PWM | Servo signal: base / shoulder / elbow / claw |
| PB6 / PB7 | I2C1 SCL / SDA, 100 kHz | Both MPU6050s, in parallel |
| PA13 / PA14 | SWDIO / SWCLK | ST-Link, plus `GND`. ST-Link `3.3V` and `RST` are not connected (OpenOCD resets over SWD). |
| PC13 | LED (active low) | On-board |
| 3.3V | Sensor supply | Both MPU6050 `Vin`, forearm `AD0`, MyoWare `VIN` |
| GND | Common ground | Every device, including the servo supply's negative |

The board is powered from its own USB port, and every other device connects only signal lines and ground, so no two
supplies ever drive the same rail.

**Servo power.** Four servos starting or stalling together draw far more current than the board's rail can supply, so
they run from a 4×AA pack through two WAGO 221 lever connectors (+ and −). A 1000 µF capacitor and a 0.1 µF ceramic capacitor sit
across the two connectors, and one wire runs from the − connector to the board's GND. Without that common ground the
servos don't move at all, even though the PWM is correct. The pack should stay above ~4.8 V (the SG92R's rated voltage) under load.

**Sensor wiring.** 3.3 V, GND, SDA and SCL are distributed through WAGO 221 connectors. Thin dupont wire is below the
connector's 0.2 mm² minimum, so fold it over or crimp a ferrule on it, and tug-test every wire: an I2C line with an
intermittent contact shows up as bursts of NACKs, timeouts and bus recoveries in the diag line.

## Toolchain (once)

Homebrew's `arm-none-eabi-gcc` ships without newlib (no `<stdint.h>`), so use Arm's own release:

```sh
curl -L -o /tmp/arm-gnu-toolchain.tar.xz \
  "https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
mkdir -p ~/.local/arm-toolchain
tar -xJf /tmp/arm-gnu-toolchain.tar.xz -C ~/.local/arm-toolchain --strip-components=1
brew install openocd
```

`cmake/arm-none-eabi-toolchain.cmake` points at `~/.local/arm-toolchain` explicitly. CI uses the x86-64 build of the
same release.

## Build, flash, start

```sh
cmake -S . -B build && cmake --build build -j          # every target; prints arm-none-eabi-size for each
cmake --build build --target flash_<target>            # flash at 0x08004000 with OpenOCD + ST-Link
```

The servo outputs of `phase3_control_loop` are off unless you build with `-DEDGENEURO_DRIVE_SERVOS=ON` (CI checks that
the default build contains no TIM3 access). That way, flashing it only to watch the MuJoCo model can't move the real
arm. `-DEDGENEURO_SERVO_MASK=<bits>` (bit 0 base … bit 3 claw) limits which servos follow the sensors, one at a time if
needed.

```sh
cmake -S . -B build-servos -DEDGENEURO_DRIVE_SERVOS=ON && cmake --build build-servos --target flash_phase3_control_loop
```

After an SWD flash the WeAct bootloader often stays in control (unplugging and replugging, or `reset run`, are both
unreliable). Start the app by jumping straight to its reset handler, then confirm:

```sh
openocd -f openocd.cfg -c "init; reset halt; reg msp [read_memory 0x08004000 32 1]; reg pc [expr {[read_memory 0x08004004 32 1] & ~1}]; reg xPSR 0x01000000; resume; exit"
python3 ../tools/check_hardware_ready.py --boot-check
```

This relies on `Reset_Handler` enabling the FPU itself. A standalone power-up with no ST-Link still goes through the
bootloader.

## Targets

| Target | What it does / proved | UART |
| --- | --- | --- |
| **`phase3_control_loop`** | **The product firmware.** 1 kHz EMG + two IMUs + (optionally) the four servos. See below. | 115200 |
| `pipeline_latency` | Per-tick latency of the generic `EdgeNeuro<>` pipeline, using the DWT cycle counter (results in the root README). | 115200 |
| `i2c_bus_scan` | Diagnostic: scans every I2C1 address; the results are read over SWD (`g_scan_bitmap`). Run by `check_hardware_ready.py --i2c-scan`. | – |
| `servo_pwm_test` | First servo on PA6: sweeps between the unit's measured limits (450–2500 µs). | – |
| `servo_limit_finder` | One servo's real range, nudged by hand (`+`/`-`). | 115200 |
| `servo_pwm_4ch_test` | All four servos, moved one at a time so a miswired channel stands out. | – |
| `servo_limit_finder_4ch` | `1`–`4` selects a servo, `+`/`-` nudges it. Used by `tools/servo_pose_4ch.py` and `tools/measure_linkage_region.py`. | 115200 |
| `blink` | Toolchain → flash → execute works at all (PC13, ~1 Hz). | – |
| `footprint_check` | The real `include/edgeneuro/` engine compiles for the target unmodified. | – |
| `heap_guard_check` | 100,000 `tick()` calls under an armed `NoHeapGuard` on the target: LED solid = no allocation. | – |
| `uart_hello` | USART2 transmit. | 9600 |
| `adc_hello` | ADC1 on PA0, software-triggered. | 9600 |
| `timer_adc_1khz` | TIM2 TRGO hardware-triggers ADC1 at exactly 1 kHz (measured report intervals 1.002–1.005 s). | 9600 |
| `i2c_mpu6050_hello` | Blocking I2C1 read of an MPU6050. | 9600 |
| `complementary_filter_hello` | Complementary filter fed by a real MPU6050. | 9600 |
| `emg_grip_control` | EMG-only grip state machine on a real MyoWare. | 9600 |

## `phase3_control_loop`

**Main loop.** TIM2 triggers ADC1 every 1 ms; each conversion is one tick. The loop never blocks:

- *IMUs.* A 14-byte MPU6050 read takes ~1.5–2 ms at 100 kHz, longer than a tick, so `ImuReader` splits it into explicit
  states and checks one hardware flag per call, alternating between the two sensors. A stuck bus is cleared the I2C way
  (UM10204 §3.1.16: up to nine SCL pulses, then STOP), and a sensor that recovers from a fault is woken again.
  `PWR_MGMT_1` is read back once a second, so a sensor that lost power and came back asleep is caught.
- *EMG.* EMA (α = 0.1) → `GripStateMachine` (on threshold, lower release threshold, 0.15 s debounce each way) →
  `SlewRateLimiter`.
- *Servos* (servo build only), every 10 ticks: 1€ filter on both raw accelerometer vectors → Path B decode → height/reach
  mapping inside the measured envelope → base hysteresis and slow follow while raising → start-up ramps
  (`include/edgeneuro/control/`). On an IMU fault nothing is written, so every servo stops where it is.
- *UART.* Output is queued (`TxRing<2048>`) and sent one byte whenever the transmitter is free. A line that doesn't fit
  is skipped whole and counted, never sent half.

**Output** (115200 8N1):

| Line | When | Content |
| --- | --- | --- |
| `tick=… grip=… gripping=… shoulder_pitch=… shoulder_roll=… elbow=… emg_min=… emg_max=… elbow_raw_a{x,y,z}=… shoulder_raw_a{x,y,z}=… shoulder_raw_g{x,y,z}=…` | every 10 ticks, if it fits | The PC uses `grip` (setpoint), `elbow` (bend between the two gravity vectors) and the upper-arm raw vector to drive the MuJoCo models, both raw vectors for its health checks, and `emg_min/max` for EMG calibration. `shoulder_pitch/roll` (complementary filter) and the upper-arm gyro are diagnostic only; nothing uses them for control |
| `EDGE -> Gripping` / `EDGE -> Released` | on a grip transition | |
| `diag …` | once a second | Per-IMU completions, NACKs, timeouts, wake results, `PWR_MGMT_1`, bus recoveries, UART skipped/dropped/overrun counts, calibrations applied/rejected/malformed |

**Commands** (host → board):

| Command | Effect |
| --- | --- |
| `O<bits>\n` | Within ~300 ms of boot only: sensors allowed to be absent (bit 0 upper arm, bit 1 forearm). Default: both required. |
| `T<on>\n` or `T<on>,<release>\n` | Set the EMG grip thresholds live. A malformed line is dropped whole. |
| `C<20 values>,<checksum>\n` | The arm calibration: four pose vectors, elbow zero, base reach, as fixed-point integers. Checksum-verified, then validated before use; otherwise the arm keeps its current calibration. Sent by `run_demo_live.py`, one byte every 2 ms (the receiver has no buffer). |
| `R` | Walk every servo slowly back to the start pose (base/shoulder/elbow 1500 µs, claw 1300 µs open), then resume following. |

**Boot failures.** If a required IMU doesn't acknowledge its wake-up write, the LED blinks a code forever: 9 = upper
arm (`0x68`), 10 = forearm (`0x69`). Read `g_wake_result_shoulder` / `g_wake_result_elbow` over SWD for the I2C failure
code (2 = address NACK).

**Servo limits** (measured on the assembled arm): base 500–2500 µs (1700 turns left), shoulder 1200–2100, elbow 500–1850,
claw 1300–1500 (travel ends at 1600; capped at 1500 in software). The shoulder × elbow combination is limited further
by the envelope in `include/edgeneuro/control/mearm_envelope_data.hpp`, generated by `tools/gen_mearm_envelope.py` from
the five measurement runs listed in its `SOURCES`.

## Known issues

- I2C runs at 100 kHz. 400 kHz reached 2028 reads/s but wasn't reliable on breadboard wiring; the constants are kept
  for soldered wiring.
- UART receive is polled one byte per loop pass, so the host must pace what it sends (2 ms per byte for the
  calibration). An RXNE interrupt with a ring buffer would remove that limit.
- The WeAct HID flashing tool doesn't work on Apple Silicon (its `hid_enumerate()` returns empty paths); flash over
  ST-Link instead.

# firmware/

Phase 1.5 feasibility check: a deliberately small step that shows the host-verified `include/edgeneuro/` engine runs on the real STM32F401 before the full Phase 3 (hardware layer, closed-loop actuation). The questions it answers and the progress are in [PRD.md](../PRD.md), Phase 1.5; this file covers building, flashing and wiring.

## Target hardware

- **MCU:** STM32F401RCT6 Black Pill, Cortex-M4F (up to 84 MHz; the firmware runs at the default 16 MHz HSI), 256 KB Flash, 64 KB SRAM.
- **Bootloader:** the board ships with the WeAct HID bootloader in the first 16 KB of Flash (`0x08000000`–`0x08003FFF`). Apps start at `0x08004000` (`linker/STM32F401RCTx_FLASH.ld`). **Do not move `FLASH ORIGIN` back to `0x08000000`**: that would overwrite the bootloader.
- **EMG sensor:** MyoWare 2.0, one channel.

## Layout

```
cmake/arm-none-eabi-toolchain.cmake   Cortex-M4F cross-compilation toolchain file
linker/STM32F401RCTx_FLASH.ld         Flash/RAM layout, app start address (0x08004000)
openocd.cfg                           ST-Link + STM32F4 target
src/startup.c                         Hand-written vector table + Reset_Handler
src/main.c                            Stage 0: blink
src/footprint_check_main.cpp          Stage 1: the real include/edgeneuro/ headers built for the target
src/no_heap_guard_target.cpp          Stage 2: bare-metal NoHeapGuard (signals a violation on the LED)
src/heap_guard_check_main.cpp         Stage 2: guarded tick() loop, pass/fail on the LED
src/uart_hello_main.c                 Stage 3a: USART2 transmit (PA2/PA3)
src/adc_hello_main.c                  Stage 3b: ADC1 on PA0, software-triggered
src/timer_adc_1khz_main.c             Stage 3c/3d: TIM2 TRGO triggers ADC1 at 1 kHz
src/i2c_mpu6050_hello_main.c          Stage 4a: I2C1 (PB6/PB7) raw accelerometer and gyroscope from an MPU6050
src/complementary_filter_hello_main.cpp  Stage 4b: ComplementaryFilter on real MPU6050 data
src/i2c_bus_scan_main.c               Diagnostic: scans every I2C1 address (tools/check_hardware_ready.py --i2c-scan)
src/complementary_filter_stress_test_main.cpp  Diagnostic: the ComplementaryFilter loop on synthetic data, no MPU6050 needed
src/emg_grip_control_main.cpp         Stage 5a: MyoWare signal drives GripStateMachine + SlewRateLimiter (no MPU6050)
src/phase3_control_loop_main.cpp      Stage 5b: EMG + IMU in one 1 kHz loop, non-blocking I2C
```

## Toolchain (once)

Homebrew's `arm-none-eabi-gcc` formula has no newlib (not even `<stdint.h>`), so use Arm's GNU toolchain release:

```sh
curl -L -o /tmp/arm-gnu-toolchain.tar.xz \
  "https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
mkdir -p ~/.local/arm-toolchain
tar -xJf /tmp/arm-gnu-toolchain.tar.xz -C ~/.local/arm-toolchain --strip-components=1
brew install dfu-util openocd
```

`cmake/arm-none-eabi-toolchain.cmake` points at `~/.local/arm-toolchain` explicitly.

## Build

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build --target blink footprint_check heap_guard_check uart_hello adc_hello timer_adc_1khz i2c_mpu6050_hello -j
```

Every target also produces a `.bin` (via `objcopy`) and prints `arm-none-eabi-size` on each build.

## Flash (ST-Link over SWD)

```sh
cmake --build build --target flash_blink            # the LED blinks at ~1 Hz
cmake --build build --target flash_footprint_check   # runs; no pass/fail signal yet
cmake --build build --target flash_heap_guard_check  # LED solid = pass, fast blink = allocation detected
cmake --build build --target flash_uart_hello        # one line per second at 9600 baud
cmake --build build --target flash_adc_hello         # one raw ADC value per second
cmake --build build --target flash_timer_adc_1khz    # one report per 1000 samples, about 1 s apart
cmake --build build --target flash_i2c_mpu6050_hello # one line of raw accel/gyro values per second
```

Each `flash_<name>` target runs `openocd -f openocd.cfg -c "program <bin> 0x08004000 verify reset exit"`, which writes from sector 1 on and leaves the bootloader in sector 0 untouched.

## Verified on the board (2026-08-14)

Stages 0 and 2 were flashed and checked on the STM32F401. The steps, to repeat after any change:

**1. Check the ST-Link connection before flashing:**

```sh
openocd -f openocd.cfg -c "init; reset halt; exit"
```

This only halts the CPU; nothing is written. A working connection prints something like:

```
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.276242
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
[stm32f4x.cpu] halted due to debug-request, current mode: Thread
```

Check the ST-Link VID:PID, a target voltage near 3.3 V and a detected `Cortex-M4`. If any is missing, check the SWDIO, SWCLK and GND wires before flashing.

**2. Flash Stage 0 (`blink`):**

```sh
cmake --build build --target flash_blink
```

OpenOCD should print `** Programming Finished **`, `** Verified OK **` and `flash size = 256 KiB`. The PC13 LED then blinks at about 1 Hz.

**3. Flash Stage 2 (`heap_guard_check`): `malloc_count == 0` on the real hardware:**

```sh
cmake --build build --target flash_heap_guard_check
```

OpenOCD prints the same as in step 2. Then the LED:
- **Solid** = pass: 100,000 guarded `tick()` calls, no allocation.
- **Fast blink** = fail: an allocation was detected in the ARM build.

**Result on this board:** both stages passed on the first flash, answering questions 1 and 2. Question 3 (1 kHz sampling) is answered in Stage 3c/3d below; question 4 (the MyoWare signal) still needs electrodes (PRD.md, Phase 1.5).

**4. Flash Stage 3a (`uart_hello`) and listen on the serial port:**

```sh
cmake --build build --target flash_uart_hello
```

The register values (AF7 on `PA2`/`PA3`, `USART_CR1`/`USART_SR`/`USART_BRR`, the baud rate) are checked against RM0368 and the STM32F401 datasheet.

Listen at 9600 baud, 8N1:

```sh
python3 -c "
import serial, time
ser = serial.Serial('/dev/tty.usbserial-0001', 9600, timeout=1)
time.sleep(0.5)
for _ in range(5):
    print(ser.readline())
"
```

(`pyserial` reads the port more reliably on macOS than `screen` or `stty` + `cat`.)

**Result:** the LED blinks and the port receives a clean, repeated `"EdgeNeuro Stage 3a: UART alive"`.

**5. Flash Stage 3b (`adc_hello`): the ADC on its own, before adding the timer:**

```sh
cmake --build build --target flash_adc_hello
```

Register values from RM0368 ch. 11. Expect one line per second, `PA0 ADC1_IN0 raw = <0~4095>`. **Result:** with the MyoWare connected and a finger on the electrodes, 1279–1316, far steadier than an open input.

**6. Flash Stage 3c/3d (`timer_adc_1khz`): timer and ADC together (question 3):**

```sh
cmake --build build --target flash_timer_adc_1khz
```

TIM2's TRGO triggers ADC1 in hardware, so `TIM2_PSC`/`TIM2_ARR` alone set the sample rate. Register values from RM0368 ch. 13 and 11. Measure the interval between reports (one per 1000 samples):

```sh
python3 -c "
import serial, time
ser = serial.Serial('/dev/tty.usbserial-0001', 9600, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()
prev = None
for _ in range(5):
    line = ser.readline()
    now = time.time()
    print(line, '' if prev is None else f'(+{now-prev:.3f}s)')
    prev = now
"
```

**Result:** **1.005 s, 1.005 s, 1.002 s**, within the HSI's own tolerance: 1 kHz sampling holds. Question 3 answered.

**7. Flash Stage 4a (`i2c_mpu6050_hello`): I2C1 read of an MPU6050:**

```sh
cmake --build build --target flash_i2c_mpu6050_hello
```

Register values from RM0368 ch. 18 and InvenSense RM-MPU-6000A-00 / PS-MPU-6000A-00. **Result:** at rest `accel_z≈16850` (≈1.03 g), `accel_x`/`y` near 0, gyro offsets in the normal range.

Lessons from the debugging:
- **Unstable UART is often wiring:** here `TXD`/`RXD` were on `A1`/`A2` instead of `A2`/`A3`.
- **A hung I2C bus shows in `I2C1_SR2.BUSY`**, read directly over SWD.
- **Multi-byte reads must time NACK/STOP with `BTF`, not `RXNE`** (RM0368 §18.3.3, N > 2 bytes); single-byte reads hide the difference.
- **When UART is unreliable, read registers and memory over SWD** (`openocd halt`, `reg pc`, `mdw <addr>`) instead of interpreting the LED.
- **`I2C_CR1.SWRST` clears a stuck `BUSY`** (RM0368 18.6.1); `i2c1_init()` now always does it.
- **To tell a faulty chip from a firmware fault, test a different I2C device:** a bus scanner and an LCD1602 backpack (PCF8574) showed the STM32 side working and both MPU6050 modules faulty.

## MyoWare 2.0 wiring

From SparkFun's MyoWare 2.0 documentation:

| MyoWare pin | To | Notes |
| --- | --- | --- |
| `VIN` | STM32 3.3V | The sensor accepts 2.27–5.47 V; at 3.3 V its 0–VIN `ENV` output matches the ADC's 0–3.3 V input. |
| `GND` | STM32 GND | Common ground. |
| `ENV` | STM32 PA0 (`ADC1_IN0`) | The envelope output, meant for an ADC input. |

Electrode placement (snap connectors, forearm flexors for grip):
- **MID**: muscle belly (mid forearm, palm side)
- **END**: toward the wrist, same muscle
- **REF**: bony or neutral spot (e.g. the elbow); a poor REF contact degrades the signal

## Power (single supply, 2026-08-18)

On the breadboard only one device supplies power; every other device connects signal lines and ground only:

- **The board:** powered from its own USB port, which gives both the 3.3 V and the 5 V rail.
- **ST-Link:** `SWCLK`, `SWDIO` and `GND` only; **not `3.3V`/`VCC`**.
- **USB-serial adapter:** `GND`, `TXD`, `RXD` only (below).
- All grounds are connected together.

A finished device would run from its own battery; this rule is for the breadboard.

## USB-serial wiring (UART for diagnostics)

**USART2** (`PA2` = TX, `PA3` = RX) is this board's standard debug console in Zephyr's board files, and does not clash with the other pins in use (`PC13` LED, `PA13`/`PA14` SWD, `PA0` MyoWare).

| USB-serial pin | To | Notes |
| --- | --- | --- |
| `GND` | STM32 GND | Common ground. |
| `TXD` | STM32 `PA3` (USART2_RX) | Crossed: adapter TX to MCU RX. |
| `RXD` | STM32 `PA2` (USART2_TX) | Crossed: adapter RX to MCU TX. |
| `VCC` (3V3/5V) | **not connected** | The board already has power; connecting VCC too would put two supplies on one rail. |

On the Mac, `ls /dev/tty.usbserial-*` shows whether the adapter enumerated, before any UART firmware runs.

**Before flashing or listening, run `python3 tools/check_hardware_ready.py`**: it checks that the ST-Link and the USB-serial adapter are present and that the port actually opens.

**If an I2C device does not respond, add `--i2c-scan`:** it flashes `firmware/src/i2c_bus_scan_main.c`, scans addresses 0–127 and reports which acknowledge and whether the bus is stuck `BUSY`. Needs only the ST-Link.

## MPU6050/GY-521 wiring (Stage 4a)

| GY-521 pin | To | Notes |
| --- | --- | --- |
| `VCC` | STM32 **5V** | The GY-521 board has its own regulator, which needs headroom, so it is fed 5 V rather than 3.3 V. |
| `GND` | STM32 GND | Common ground. |
| `SCL` | STM32 `PB6` (I2C1_SCL, AF4) | Datasheet Table 9; no clash with `PA0`/`PA2`/`PA3`/`PA13`/`PA14`. |
| `SDA` | STM32 `PB7` (I2C1_SDA, AF4) | As above. |
| `XDA`/`XCL`/`ADO`/`INT` | not connected | Not used. |

**Breadboard:** seat all eight pins fully; a loose module holds the bus `BUSY` and the firmware appears to hang. `openocd -f openocd.cfg -c "init" -c "halt" -c "mdw 0x40005418 1" -c "resume" -c "shutdown"` reads `I2C1_SR2`; `BUSY` is bit 1.

`WHO_AM_I` (0x75) is documented as `0x68`, but these GY-521 boards consistently return `0x72` (a compatible chip); the firmware accepts both.

**Status (2026-08-18): neither GY-521 module communicates, also on 5 V; the fault is in the modules.** The I2C1 firmware still works with the LCD1602 on the single-supply wiring. An Adafruit MPU-6050 is ordered (PRD.md, Stage 4b).

## Stage 5a: EMG grip control (`emg_grip_control`)

```sh
cmake --build build --target flash_emg_grip_control
```

The EMG half of the Phase 3 design (PRD.md §3), without the MPU6050: TIM2 + ADC1 read the MyoWare `ENV` at 1 kHz into `GripStateMachine` (`include/edgeneuro/control/grip_state_machine.hpp`) and `SlewRateLimiter`. Wiring as in the MyoWare and USB-serial sections, `ENV → PA0`.

**Watch it live with `python3 tools/watch_myoware_uart.py`.**

**Threshold calibration (`kCalibrationEnabled`, off by default):** a compile-time switch at the top of `emg_grip_control_main.cpp`. Off, the board boots with `kFallbackThreshold` (2037, the last calibration). On, it guides a calibration over UART: each phase (relaxed, then clenched) starts when a byte arrives, e.g. `serial.Serial(port, 9600).write(b'\n')`, then samples for 3 s. It prints `relaxed_max`, `contracted_min` and the `threshold`; copy the new value into `kFallbackThreshold`.

**Result (2026-08-18):** with real electrodes, relaxed `~434–495`, clenched `~3700+`; the existing `threshold=1220` sits between them. Repeated clench/relax cycles give clean `EDGE -> Gripping` / `EDGE -> Released` transitions, and the natural decay on relaxing does not trigger early.

## Stage 5b: EMG + IMU in one loop (`phase3_control_loop`)

```sh
cmake --build build --target flash_phase3_control_loop
```

The Stage 5a EMG loop and `ComplementaryFilter` in one 1 kHz main loop. `ImuReader` splits a 14-byte I2C read into explicit states and checks one flag per call, so EMG sampling never waits. Wiring: the MyoWare (`ENV → PA0`) and the MPU6050 (`SCL → PB6`, `SDA → PB7`).

**`kImuTargetAddr` is set to the LCD1602 (`0x27`), not an MPU6050 (`0x68`),** while both MPU6050s are faulty: the LCD exercises the same I2C sequence, so connect it to PB6/PB7 for now. Its `roll_x1000`/`pitch_x1000` are not orientation, only a check that the protocol runs.

**Result (2026-08-20/21):** with `step()` called once per EMG tick, only 47–48 reads/s completed; called on every loop pass, **592–593 reads/s**, close to the bus limit, with the EMG tick unaffected.

**Next (with a working MPU6050):** set `kImuTargetAddr` back to `0x68`; try 400 kHz I2C, which the MPU6050 supports.

**Smoothing (2026-08-21):** `roll_smoother`/`pitch_smoother` (`IirFilter` as a one-pole EMA, `kSmoothAlpha=0.5`) on `roll`/`pitch`. `kSmoothAlpha` is a starting value, to be tuned on real sensor noise.

**`ImuReader` recovers from a timeout:** besides `STOP`, it does a full `SWRST` and re-configuration, as `i2c1_init()` does, so a temporary `BUSY` hang clears itself. `g_timeout_count`, `g_imu_state_at_timeout`, `g_sr1_at_timeout`, `g_sr2_at_timeout` can be read over SWD.

## Known issues

- **The WeAct HID flashing tool does not work on Apple Silicon** (`hid_enumerate()` returns empty device paths). Flash over ST-Link with OpenOCD instead.
- 1 kHz sampling is done (Stages 3a–3d). No DMA: polling `EOC` is enough for one channel at 1 kHz.

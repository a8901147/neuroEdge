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
cmake --build build --target blink footprint_check heap_guard_check uart_hello -j
```

Every target also produces a `.bin` (via `objcopy`) and prints `arm-none-eabi-size` on each build.

## Flash (ST-Link over SWD)

```sh
cmake --build build --target flash_blink            # the LED blinks at ~1 Hz
cmake --build build --target flash_footprint_check   # runs; no pass/fail signal yet
cmake --build build --target flash_heap_guard_check  # LED solid = pass, fast blink = allocation detected
cmake --build build --target flash_uart_hello        # one line per second at 9600 baud
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

**Result on this board:** both stages passed on the first flash (Stage 0 blinks, Stage 2 stays solid). Questions 1 and 2 are answered; the ADC timing and the MyoWare signal need UART output (PRD.md, Phase 1.5).

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

## USB-serial wiring (UART for diagnostics)

**USART2** (`PA2` = TX, `PA3` = RX) is this board's standard debug console in Zephyr's board files, and does not clash with the other pins in use (`PC13` LED, `PA13`/`PA14` SWD, `PA0` MyoWare).

| USB-serial pin | To | Notes |
| --- | --- | --- |
| `GND` | STM32 GND | Common ground. |
| `TXD` | STM32 `PA3` (USART2_RX) | Crossed: adapter TX to MCU RX. |
| `RXD` | STM32 `PA2` (USART2_TX) | Crossed: adapter RX to MCU TX. |
| `VCC` (3V3/5V) | **not connected** | The board already has power; connecting VCC too would put two supplies on one rail. |

On the Mac, `ls /dev/tty.usbserial-*` shows whether the adapter enumerated, before any UART firmware runs.

## Known issues

- **The WeAct HID flashing tool does not work on Apple Silicon** (`hid_enumerate()` returns empty device paths). Flash over ST-Link with OpenOCD instead.
- The timer/ADC firmware for 1 kHz sampling is not written yet; its register values are checked against RM0368 first.

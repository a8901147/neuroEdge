# firmware/

Phase 1.5 feasibility spike: a scoped, deliberately small check that the Host-validated `include/edgeneuro/` engine actually works on the real STM32F401 target before committing to full Phase 3 (HAL layer, closed-loop actuation). See [PRD.md](../PRD.md) Phase 1.5 for the full rationale, the four unknowns this is answering, and a running log of what's been found so far — this file covers day-to-day build/flash commands and hardware wiring only.

## Target hardware

- **MCU**: STM32F401RCT6 "Black Pill" — Cortex-M4F @ 84MHz (currently unconfigured; firmware runs on the default 16MHz HSI), 256KB Flash, 64KB SRAM.
- **Bootloader**: this specific board ships pre-flashed with WeAct Studio's HID bootloader, occupying the first 16KB of Flash (`0x08000000`–`0x08003FFF`). Our application starts at `0x08004000` — see `linker/STM32F401RCTx_FLASH.ld`. **Do not change `FLASH ORIGIN` back to `0x08000000`** — that would overwrite the bootloader.
- **EMG sensor**: MyoWare 2.0, 1 channel.

## Layout

```
cmake/arm-none-eabi-toolchain.cmake   Cortex-M4F cross-compilation toolchain file
linker/STM32F401RCTx_FLASH.ld         Flash/RAM layout, application offset (0x08004000)
openocd.cfg                           ST-Link + STM32F4 target config
src/startup.c                         Hand-written vector table + Reset_Handler (no CMSIS startup file)
src/main.c                            Stage 0: blink
src/footprint_check_main.cpp          Stage 1: real include/edgeneuro/* compiled for this target
src/no_heap_guard_target.cpp          Stage 2: bare-metal NoHeapGuard (LED violation signal, not abort())
src/heap_guard_check_main.cpp         Stage 2: runs the armed tick() loop, reports pass/fail via LED
```

## Toolchain setup (one-time)

The Homebrew `arm-none-eabi-gcc` formula ships the compiler only, without newlib (no `<stdint.h>` even). Use the official ARM GNU Toolchain tarball instead:

```sh
curl -L -o /tmp/arm-gnu-toolchain.tar.xz \
  "https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
mkdir -p ~/.local/arm-toolchain
tar -xJf /tmp/arm-gnu-toolchain.tar.xz -C ~/.local/arm-toolchain --strip-components=1
brew install dfu-util openocd
```

`cmake/arm-none-eabi-toolchain.cmake` points at `~/.local/arm-toolchain` explicitly, not whatever's on `PATH`.

## Build

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build --target blink footprint_check heap_guard_check -j
```

Each target also produces a `.bin` (via `objcopy`) and prints `arm-none-eabi-size` output on every build — Flash/SRAM footprint is visible without a separate step.

## Flash (needs ST-Link connected via SWD)

```sh
cmake --build build --target flash_blink            # then watch for a slow (~1Hz) LED blink
cmake --build build --target flash_footprint_check   # proves it doesn't crash; no pass/fail signal yet
cmake --build build --target flash_heap_guard_check  # LED solid ON = pass, fast blink = malloc_count violation
```

Each `flash_<name>` target runs `openocd -f openocd.cfg -c "program <bin> 0x08004000 verify reset exit"` — writes only Sector 1 onward, leaving the bootloader (Sector 0) untouched.

**Not yet tested against real hardware** — ST-Link is still in transit as of this writing. All three targets build and link cleanly; see PRD.md for current Flash/SRAM numbers per stage.

## MyoWare 2.0 wiring (for Stage 4, once ST-Link + USB-TTL both arrive)

From SparkFun's official MyoWare 2.0 documentation:

| MyoWare pin | Connects to | Notes |
| --- | --- | --- |
| `VIN` | STM32 3.3V | Sensor accepts 2.27V–5.47V; using 3.3V means `ENV`'s 0–VIN output range lines up exactly with the STM32 ADC's 0–3.3V input range — no level shifting needed. |
| `GND` | STM32 GND | Common ground, required regardless of the above. |
| `ENV` | STM32 PA0 (`ADC1_IN0`) | Envelope-detected output — SparkFun's recommended pin for direct ADC input (vs. the raw/rectified test pads on the underside, meant for advanced custom post-processing). |

Electrode placement (bottom-side snap connectors, for forearm flexor / grasp detection):
- **MID** — muscle belly (mid-forearm, volar/palm side)
- **END** — toward the wrist, same muscle
- **REF** — a bony/neutral site (e.g. elbow) — SparkFun specifically warns a poor REF contact degrades signal quality

## Known issues

- **WeAct's official HID-bootloader flashing tool doesn't work on this Apple Silicon Mac** — traced to `hid_enumerate()` returning an empty device path (deep IOKit compatibility issue with this 2019-era tool, not something worth patching further). Flashing goes through ST-Link + OpenOCD instead; see PRD.md Phase 1.5 for the full investigation.
- Timer/ADC/DMA firmware for Stage 3 (1kHz sampling) and the UART driver needed for Stage 3/4 diagnostics are **not written yet** — deliberately deferred until real register-level values (timer prescalers, ADC trigger-source encoding, USART baud settings) can be checked against RM0368 or verified on hardware, rather than shipped as best-effort guesses.

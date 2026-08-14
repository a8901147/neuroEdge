cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-toolchain.cmake)
project(edgeneuro_firmware C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F401RCTx_FLASH.ld)
set(CMAKE_EXE_LINKER_FLAGS
    "-T${LINKER_SCRIPT} -nostartfiles -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/firmware.map")

# Application start address, after WeAct Studio's pre-flashed HID
# bootloader (first 16KB of Flash) -- must match linker/STM32F401RCTx_FLASH.ld
# and openocd.cfg's flash address.
set(APP_FLASH_ADDRESS 0x08004000)

include(FetchContent)
FetchContent_Declare(
    cmsis_device_f4
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f4.git
    GIT_TAG v2.6.9
)
FetchContent_Populate(cmsis_device_f4)

FetchContent_Declare(
    cmsis_core
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-core.git
    GIT_TAG v5.9.0
)
FetchContent_Populate(cmsis_core)

# Defines <name>, <name>.bin (via objcopy, printing arm-none-eabi-size on
# every build), and a flash_<name> target (openocd, needs ST-Link attached
# -- not available until Phase 1.5's ST-Link arrives, see PRD.md).
function(firmware_add_target name)
    add_executable(${name} ${ARGN})
    target_compile_definitions(${name} PRIVATE STM32F401xC)
    target_include_directories(${name} PRIVATE
        ${cmsis_device_f4_SOURCE_DIR}/Include
        ${cmsis_core_SOURCE_DIR}/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
    )
    target_compile_options(${name} PRIVATE -Os -ffunction-sections -fdata-sections -Wall -Wextra)

    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> $<TARGET_FILE:${name}>.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${name}>
    )

    add_custom_target(flash_${name}
        COMMAND openocd -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd.cfg
                -c "program $<TARGET_FILE:${name}>.bin ${APP_FLASH_ADDRESS} verify reset exit"
        DEPENDS ${name}
        COMMENT "Flashing ${name} via ST-Link + OpenOCD (needs board connected)"
        VERBATIM
    )
endfunction()

# Stage 0: blink -- proves toolchain -> flash -> execute works at all.
firmware_add_target(blink src/startup.c src/main.c)

# Stage 1: compile the REAL Phase 1 engine (include/edgeneuro/) for this
# target and report its actual Flash/SRAM footprint. No CsvSignalProvider
# here (needs a filesystem) -- a minimal stub Provider stands in for it,
# same shape the real Stm32AdcProvider will eventually have.
firmware_add_target(footprint_check src/startup.c src/footprint_check_main.cpp)

# Stage 2: answers unknown #1 -- does NoHeapGuard actually stay at
# malloc_count == 0 on this target's real ARM GCC codegen, not just Host's.
# LED solid ON = pass, fast blink = a real allocation was caught.
firmware_add_target(heap_guard_check
    src/startup.c
    src/no_heap_guard_target.cpp
    src/heap_guard_check_main.cpp
)

# Stage 3a: USART2 polling transmit-only (PA2/PA3), isolated from Timer/ADC
# -- proves the UART wiring + register config works on its own before
# Timer+ADC 1kHz sampling (the rest of unknown #3) is added on top.
firmware_add_target(uart_hello src/startup.c src/uart_hello_main.c)

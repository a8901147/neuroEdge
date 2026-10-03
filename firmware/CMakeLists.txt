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

# Stage 3b: ADC1 channel 0 (PA0), software-triggered continuous conversion,
# polled -- isolates the ADC from Timer-driven periodic triggering. Reports
# readings over USART2 (reuses Stage 3a's now-verified UART setup).
firmware_add_target(adc_hello src/startup.c src/adc_hello_main.c)

# Stage 3c/3d: TIM2 drives ADC1 at a hardware-timed 1kHz rate via TRGO --
# this is unknown #3's real test (Stage 3b only proved software-triggered
# ADC reads work, not a timed 1kHz rate).
firmware_add_target(timer_adc_1khz src/startup.c src/timer_adc_1khz_main.c)

# Stage 4a: I2C1 (PB6/PB7) polled master driver reading an MPU6050/GY-521,
# isolated from the ComplementaryFilter integration that comes next.
firmware_add_target(i2c_mpu6050_hello src/startup.c src/i2c_mpu6050_hello_main.c)

# Stage 4b: the Host-tested include/edgeneuro/fusion/complementary_filter.hpp
# fused with real MPU6050 data (reuses Stage 4a's verified I2C1 read path).
firmware_add_target(complementary_filter_hello src/startup.c src/complementary_filter_hello_main.cpp)

# Permanent diagnostic tool (not a pipeline stage): scans the full I2C1
# 7-bit address space and reports which addresses ACK. Driven by
# tools/check_hardware_ready.py --i2c-scan -- see that script and this
# target's own file header for why this exists.
firmware_add_target(i2c_bus_scan src/startup.c src/i2c_bus_scan_main.c)

# Stage 5a: the EMG half of Phase 3's split control loop (GripStateMachine +
# SlewRateLimiter) on real MyoWare hardware, independent of the
# still-blocked MPU6050 side. See this target's own file header.
firmware_add_target(emg_grip_control src/startup.c src/emg_grip_control_main.cpp)

# Stage 5b: EMG + IMU combined into the real Phase 3 loop shape, with a
# non-blocking I2C1 read state machine (necessary since a blocking 14-byte
# read takes longer than one EMG tick). Targets the LCD1602 stand-in
# address until a working MPU6050 arrives -- see this target's own file
# header for why.
firmware_add_target(phase3_control_loop src/startup.c src/phase3_control_loop_main.cpp)

# The real-MEArm servo outputs (TIM3 CH1-4 on PA6/PA7/PB0/PB1) are OFF by
# default (2026-09-26 review): flashing phase3_control_loop -- needed just to
# run `run_demo_live.py --mearm` against the MuJoCo model -- must not also move
# the physical arm. The firmware's servo mapping still drives shoulder/elbow
# independently (ignoring the linkage coupling, SESSION_LOG TODO C) and from
# different signals than the tested Path B, so turn it on only deliberately:
#   cmake -S firmware -B firmware/build -DEDGENEURO_DRIVE_SERVOS=ON
option(EDGENEURO_DRIVE_SERVOS "Drive the 4 MEArm servos from phase3_control_loop" OFF)
if(EDGENEURO_DRIVE_SERVOS)
    target_compile_definitions(phase3_control_loop PRIVATE EDGENEURO_DRIVE_SERVOS=1)
    # Which servos follow the sensors, the others hold rest (2026-09-28: one at a time to find a bad servo / weak
    # supply): bit0 base, bit1 shoulder, bit2 elbow, bit3 claw. e.g. -DEDGENEURO_SERVO_MASK=2 = shoulder only.
    set(EDGENEURO_SERVO_MASK "15" CACHE STRING "servos that follow the sensors (bit0 base .. bit3 claw)")
    target_compile_definitions(phase3_control_loop PRIVATE EDGENEURO_SERVO_MASK=${EDGENEURO_SERVO_MASK})
endif()

# Stage 7a (Phase 4 bring-up): 50Hz hobby-servo PWM on PA6 (TIM3_CH1),
# sweeping pulse width so a single SG90 can be visually confirmed to move
# end-to-end, before any of this is wired into the real control loop.
firmware_add_target(servo_pwm_test src/startup.c src/servo_pwm_test_main.c)

# Stage 7b: interactive follow-up to servo_pwm_test -- nudges the same
# PA6/TIM3_CH1 PWM pulse width up/down by UART command ('+'/'-') so a
# servo's real safe range can be found by ear/eye instead of guessed.
firmware_add_target(servo_limit_finder src/startup.c src/servo_limit_finder_main.c)

# Stage 7c: drives all 4 of TIM3's PWM channels (PA6/PA7/PB0/PB1) at once,
# one MEArm servo per channel (base/shoulder/elbow/claw, bottom-to-top --
# see the file's own header comment for why that's the assignment).
# Moves one channel at a time so a miswired channel shows up as "this one
# servo didn't move," not 4 servos moving with no way to tell which is
# wrong.
firmware_add_target(servo_pwm_4ch_test src/startup.c src/servo_pwm_4ch_test_main.c)

# Stage 7d: interactive 4-channel follow-up -- send '1'-'4' over UART to
# select a mounted servo (base/shoulder/elbow/claw), then '+'/'-' to find
# ITS real usable range once actually mounted on the assembled MEArm (the
# linkage geometry caps this well below the bare servo's own limit).
firmware_add_target(servo_limit_finder_4ch src/startup.c src/servo_limit_finder_4ch_main.c)

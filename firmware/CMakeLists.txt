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

# ---------------------------------------------------------------------------
# Stage 0: blink -- proves toolchain -> flash -> execute works at all.
# ---------------------------------------------------------------------------
add_executable(blink src/startup.c src/main.c)
target_compile_definitions(blink PRIVATE STM32F401xC)
target_include_directories(blink PRIVATE
    ${cmsis_device_f4_SOURCE_DIR}/Include
    ${cmsis_core_SOURCE_DIR}/Include
)
target_compile_options(blink PRIVATE -Os -ffunction-sections -fdata-sections -Wall -Wextra)

add_custom_command(TARGET blink POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:blink> $<TARGET_FILE:blink>.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:blink>
)

# ---------------------------------------------------------------------------
# Stage 1: compile the REAL Phase 1 engine (include/edgeneuro/) for this
# target and report its actual Flash/SRAM footprint. No CsvSignalProvider
# here (needs a filesystem) -- a minimal stub Provider stands in for it,
# same shape the real Stm32AdcProvider will eventually have.
# ---------------------------------------------------------------------------
add_executable(footprint_check
    src/startup.c
    src/footprint_check_main.cpp
)
target_compile_definitions(footprint_check PRIVATE STM32F401xC)
target_include_directories(footprint_check PRIVATE
    ${cmsis_device_f4_SOURCE_DIR}/Include
    ${cmsis_core_SOURCE_DIR}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_compile_options(footprint_check PRIVATE -Os -ffunction-sections -fdata-sections -Wall -Wextra)

add_custom_command(TARGET footprint_check POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:footprint_check> $<TARGET_FILE:footprint_check>.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:footprint_check>
)

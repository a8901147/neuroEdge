# Cross-compilation toolchain for STM32F401RCT6 (Cortex-M4F).
# Usage: cmake -S firmware -B firmware/build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# The Homebrew `arm-none-eabi-gcc` formula ships the compiler only, without
# newlib (no libc headers at all -- even <stdint.h> fails). Use the official
# ARM GNU Toolchain tarball instead (has full newlib), extracted to a
# non-sudo user-local path -- see PRD.md Phase 1.5 for how it got there.
set(ARM_TOOLCHAIN_ROOT "$ENV{HOME}/.local/arm-toolchain")
set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_ROOT}/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_ROOT}/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_ROOT}/bin/arm-none-eabi-gcc)
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_ROOT}/bin/arm-none-eabi-objcopy CACHE FILEPATH "")
set(CMAKE_SIZE ${ARM_TOOLCHAIN_ROOT}/bin/arm-none-eabi-size CACHE FILEPATH "")

# No OS on the target: skip the linker-availability test CMake normally runs
# (it tries to build+run a test executable, which can't run on the host).
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${CPU_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${CPU_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

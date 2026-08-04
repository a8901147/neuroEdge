cmake_minimum_required(VERSION 3.20)
project(EdgeNeuro CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug)
endif()

option(EDGENEURO_ENABLE_HEAP_GUARD "Compile the global operator new/delete allocation counter" ON)
option(EDGENEURO_BUILD_TESTS "Build Catch2 unit tests" ON)
option(EDGENEURO_BUILD_BENCHMARKS "Build Google Benchmark suite" ON)
set(EDGENEURO_SANITIZER "none" CACHE STRING "Sanitizer to build with: none | address-undefined | thread")
set_property(CACHE EDGENEURO_SANITIZER PROPERTY STRINGS none address-undefined thread)

if(EDGENEURO_SANITIZER STREQUAL "address-undefined")
    if(EDGENEURO_ENABLE_HEAP_GUARD)
        message(FATAL_ERROR
            "NoHeapGuard's operator-new override conflicts with ASan's own allocator "
            "interception. Configure with -DEDGENEURO_ENABLE_HEAP_GUARD=OFF for sanitizer builds.")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=address,undefined)
elseif(EDGENEURO_SANITIZER STREQUAL "thread")
    if(EDGENEURO_ENABLE_HEAP_GUARD)
        message(FATAL_ERROR
            "NoHeapGuard's operator-new override conflicts with TSan's own allocator "
            "interception. Configure with -DEDGENEURO_ENABLE_HEAP_GUARD=OFF for sanitizer builds.")
    endif()
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=thread)
elseif(NOT EDGENEURO_SANITIZER STREQUAL "none")
    message(FATAL_ERROR "Unknown EDGENEURO_SANITIZER value: ${EDGENEURO_SANITIZER}")
endif()

# ---------------------------------------------------------------------------
# Core engine: header-only, zero third-party dependencies.
# ---------------------------------------------------------------------------
add_library(edgeneuro_core INTERFACE)
target_include_directories(edgeneuro_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(edgeneuro_core INTERFACE cxx_std_20)

set(EDGENEURO_WARNING_FLAGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# Optional allocation counter — only compiled into debug-heapguard / release-bench.
if(EDGENEURO_ENABLE_HEAP_GUARD)
    add_library(edgeneuro_heap_guard STATIC src/no_heap_guard.cpp)
    target_link_libraries(edgeneuro_heap_guard PUBLIC edgeneuro_core)
    target_compile_definitions(edgeneuro_core INTERFACE EDGENEURO_HEAP_GUARD_ENABLED=1)
endif()

# ---------------------------------------------------------------------------
# Demo: terminal oscilloscope for the <1,6> wearable fusion mode.
# ---------------------------------------------------------------------------
add_executable(edgeneuro_demo src/main.cpp)
target_link_libraries(edgeneuro_demo PRIVATE edgeneuro_core)
target_compile_options(edgeneuro_demo PRIVATE ${EDGENEURO_WARNING_FLAGS})
if(TARGET edgeneuro_heap_guard)
    target_link_libraries(edgeneuro_demo PRIVATE edgeneuro_heap_guard)
endif()

# ---------------------------------------------------------------------------
# Third-party test/benchmark frameworks (setup-time allocation only; never
# linked into the hot loop itself).
# ---------------------------------------------------------------------------
include(FetchContent)

if(EDGENEURO_BUILD_TESTS)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.5.4
    )
    FetchContent_MakeAvailable(Catch2)

    enable_testing()
    file(GLOB EDGENEURO_TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    add_executable(edgeneuro_tests ${EDGENEURO_TEST_SOURCES})
    target_link_libraries(edgeneuro_tests PRIVATE edgeneuro_core Catch2::Catch2WithMain)
    target_compile_options(edgeneuro_tests PRIVATE ${EDGENEURO_WARNING_FLAGS})
    target_compile_definitions(edgeneuro_tests PRIVATE
        EDGENEURO_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
    if(TARGET edgeneuro_heap_guard)
        target_link_libraries(edgeneuro_tests PRIVATE edgeneuro_heap_guard)
    endif()
    find_package(Threads REQUIRED)
    target_link_libraries(edgeneuro_tests PRIVATE Threads::Threads)

    include(${catch2_SOURCE_DIR}/extras/Catch.cmake)
    catch_discover_tests(edgeneuro_tests)
endif()

if(EDGENEURO_BUILD_BENCHMARKS)
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googlebenchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.8.4
    )
    FetchContent_MakeAvailable(googlebenchmark)

    add_executable(edgeneuro_bench benchmarks/bench_pipeline.cpp)
    target_link_libraries(edgeneuro_bench PRIVATE edgeneuro_core benchmark::benchmark)
    target_compile_options(edgeneuro_bench PRIVATE ${EDGENEURO_WARNING_FLAGS})
    if(TARGET edgeneuro_heap_guard)
        target_link_libraries(edgeneuro_bench PRIVATE edgeneuro_heap_guard)
    endif()
endif()

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
option(EDGENEURO_BUILD_GUI_DEMO "Build the ImGui+ImPlot graphical oscilloscope demo" OFF)
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
# Phase 2: CSV-replaying "twin" of the real firmware control loop
# (GripStateMachine + ComplementaryFilter + SlewRateLimiter, bypassing
# Pipeline/EdgeNeuro<> -- see the file's own header comment), emitting one
# line/tick to stdout for tools/mujoco_bridge/run_demo.py to consume.
# ---------------------------------------------------------------------------
add_executable(edgeneuro_mujoco_bridge_demo src/mujoco_bridge_demo.cpp)
target_link_libraries(edgeneuro_mujoco_bridge_demo PRIVATE edgeneuro_core)
target_compile_options(edgeneuro_mujoco_bridge_demo PRIVATE ${EDGENEURO_WARNING_FLAGS})

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
    # PROPERTIES SKIP_RETURN_CODE 4 (2026-09-07): a Catch2 v3 test binary
    # exits with code 4 specifically when EVERY test case in that run was
    # skipped (Catch::Session::run(), catch_session.cpp) -- e.g.
    # test_complementary_filter_real_data.cpp/test_real_data_integration.cpp
    # SKIP()ping themselves when the real (gitignored, 5.48GB) EMG-EPN-612
    # dataset isn't present, which is expected and correct on CI or any
    # machine that hasn't downloaded it. Without this property CTest
    # doesn't know 4 means "skipped" and reports it as a plain failure --
    # caught by this project's own CI on its first run.
    catch_discover_tests(edgeneuro_tests PROPERTIES SKIP_RETURN_CODE 4)
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

# ---------------------------------------------------------------------------
# Optional graphical demo: ImGui + ImPlot native window, real line plots
# instead of the terminal ASCII bar. Off by default — pulls in GLFW/OpenGL
# and roughly triples configure time, so it's opt-in
# (-DEDGENEURO_BUILD_GUI_DEMO=ON). Everything here is Host-side
# visualization scaffolding, same as edgeneuro_demo: NoHeapGuard is armed
# only around engine.tick(), never around ImGui/ImPlot rendering.
# ---------------------------------------------------------------------------
if(EDGENEURO_BUILD_GUI_DEMO)
    find_package(OpenGL REQUIRED)

    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
    )
    FetchContent_MakeAvailable(glfw)

    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.90.9
    )
    FetchContent_MakeAvailable(imgui)

    FetchContent_Declare(
        implot
        GIT_REPOSITORY https://github.com/epezent/implot.git
        GIT_TAG v0.16
    )
    FetchContent_MakeAvailable(implot)

    # Neither imgui nor implot ship a CMakeLists.txt for library consumption
    # (they're meant to be dropped into your own build) — compile their
    # sources directly into one static lib.
    add_library(imgui_implot STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
        ${implot_SOURCE_DIR}/implot.cpp
        ${implot_SOURCE_DIR}/implot_items.cpp
    )
    target_include_directories(imgui_implot PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
        ${implot_SOURCE_DIR}
    )
    target_link_libraries(imgui_implot PUBLIC glfw OpenGL::GL)
    target_compile_definitions(imgui_implot PUBLIC GL_SILENCE_DEPRECATION)

    add_executable(edgeneuro_gui_demo src/gui_demo.cpp)
    target_link_libraries(edgeneuro_gui_demo PRIVATE edgeneuro_core imgui_implot)
    target_compile_options(edgeneuro_gui_demo PRIVATE ${EDGENEURO_WARNING_FLAGS})
    if(TARGET edgeneuro_heap_guard)
        target_link_libraries(edgeneuro_gui_demo PRIVATE edgeneuro_heap_guard)
    endif()
endif()

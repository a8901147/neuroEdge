# EdgeNeuro

Modular, zero-allocation, real-time BCI/neuroprosthetic signal-processing engine in C++20. See [PRD.md](PRD.md) for the full product/architecture spec. This README covers day-to-day build/test/run commands only.

Currently **Phase 1**: the host-side C++20 algorithmic core (Provider → Filter → Feature → Classifier), validated for `malloc_count == 0` and sub-microsecond per-sample latency. No MuJoCo (Phase 2) or STM32 (Phase 3/4) code exists yet by design — see PRD §6 for the quality-gate discipline.

## Layout

```
include/edgeneuro/   Header-only engine: concepts, ring buffer, filters, features, classifiers, providers, pipeline
src/                 no_heap_guard.cpp (allocation counter) + main.cpp (terminal demo) + gui_demo.cpp (ImGui/ImPlot demo)
tests/               Catch2 unit + integration tests
benchmarks/          Google Benchmark suite (<1,6> wearable fusion, <32,0> HD-sEMG stress)
data/                Synthetic CSV fixtures (not real Ninapro data — see tools/generate_sample_data.py)
```

## Build

Four mutually exclusive [CMake Presets](CMakePresets.json), each mapping to one PRD §5 validation row. Don't mix `EDGENEURO_ENABLE_HEAP_GUARD` with a sanitizer in the same binary — ASan/TSan install their own allocator interceptors that conflict with `NoHeapGuard`'s global `operator new` override (CMake will refuse the combination).

| Preset | Purpose |
| --- | --- |
| `debug-heapguard` | Debug build, `NoHeapGuard` armed — proves `malloc_count == 0` |
| `sanitize-asan-ubsan` | ASan + UBSan — undefined behavior, no heap guard |
| `sanitize-tsan` | TSan — RingBuffer concurrency correctness (separate binary; ASan/TSan can't coexist) |
| `release-bench` | Release + `NoHeapGuard` + Google Benchmark |

```sh
cmake --preset debug-heapguard
cmake --build --preset debug-heapguard -j
```

Swap `debug-heapguard` for any other preset name to switch configs.

## Test

```sh
ctest --preset debug-heapguard          # or sanitize-asan-ubsan / sanitize-tsan
ctest --test-dir build/release-bench    # release-bench has no dedicated test preset
```

## Coverage

Source-based coverage via `llvm-cov` (Xcode Command Line Tools on macOS; plain `llvm-profdata`/`llvm-cov` on Linux — drop the `xcrun` prefix there). Separate build dir, not one of the four presets, since it needs its own compiler flags:

```sh
cmake -S . -B build/coverage \
  -DCMAKE_BUILD_TYPE=Debug \
  -DEDGENEURO_ENABLE_HEAP_GUARD=ON \
  -DEDGENEURO_BUILD_BENCHMARKS=OFF \
  -DEDGENEURO_BUILD_GUI_DEMO=OFF \
  -DCMAKE_CXX_FLAGS="-fprofile-instr-generate -fcoverage-mapping" \
  -DCMAKE_EXE_LINKER_FLAGS="-fprofile-instr-generate"
cmake --build build/coverage -j --target edgeneuro_tests

cd build/coverage
mkdir -p coverage_raw
LLVM_PROFILE_FILE="coverage_raw/edgeneuro-%p.profraw" ./edgeneuro_tests
xcrun llvm-profdata merge -sparse coverage_raw/*.profraw -o coverage.profdata
xcrun llvm-cov report ./edgeneuro_tests \
  -instr-profile=coverage.profdata \
  -ignore-filename-regex='_deps/|/tests/'
```

The `-ignore-filename-regex` scopes the report to our own `include/edgeneuro/` + `src/` — otherwise it'd also count FetchContent'd Catch2/benchmark source and the test files themselves. Swap `report` for `show <path/to/file>` to see line-by-line annotated source for one file. See PRD §5 for the current measured numbers and what's structurally unreachable.

## Benchmark

```sh
cmake --preset release-bench && cmake --build --preset release-bench -j
./build/release-bench/edgeneuro_bench
```

Validates PRD §5's `<32,0>` continuous latency `< 0.1ms` target and `malloc_count == 0` for both `<1,6>` and `<32,0>` modes.

## Demo

Terminal oscilloscope replaying the synthetic `<1,6>` wearable-fusion CSV at a real 1kHz pace — live EMG amplitude bar, decoded gesture, per-tick latency, running `malloc_count`.

```sh
python3 tools/generate_sample_data.py   # regenerate data/*.csv if needed
./build/debug-heapguard/edgeneuro_demo data/wearable_1emg_6imu.csv
```

### Graphical demo (ImGui + ImPlot)

Same engine and CSV, rendered as a real line chart in a native window instead of an ASCII bar. Off by default (pulls in GLFW/OpenGL via FetchContent) — enable with `-DEDGENEURO_BUILD_GUI_DEMO=ON`:

```sh
cmake -S . -B build/gui-demo -DEDGENEURO_BUILD_GUI_DEMO=ON -DEDGENEURO_BUILD_TESTS=OFF -DEDGENEURO_BUILD_BENCHMARKS=OFF
cmake --build build/gui-demo -j
./build/gui-demo/edgeneuro_gui_demo data/wearable_1emg_6imu.csv
```

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

## Real-dataset compatibility (EMG-EPN-612)

`tools/convert_epn612.py` converts [EMG-EPN-612](https://zenodo.org/records/4421500) (Myo armband) per-user JSON recordings into `CsvSignalProvider`'s CSV layout — `EdgeNeuro<8, 10>` (8 EMG channels + accelerometer/gyroscope/quaternion). Verified end-to-end against real downloaded data (not just a synthetic fixture): the converter, and the resulting CSV read back through the real C++ `CsvSignalProvider`, both confirmed correct on `trainingJSON/user1/user1.json` (298,710 rows, values cross-checked between the Python converter's output and the C++ parser's output).

Two things this confirmed about real hardware, not assumptions:

- **EMG and IMU are independently-clocked streams that don't even divide evenly** — one user1 recording had 992 EMG samples against 249 IMU samples (~3.98:1, not a clean 4:1 despite the dataset's documented ~200Hz/~50Hz headline rates). The converter resamples the slower IMU stream up to the EMG sample count with zero-order hold (repeat the last reading), indexed by length ratio rather than an assumed fixed Hz — this is also what real Phase 3 firmware would do in its main loop (run at the EMG ADC's rate, only refresh the IMU reading every Nth tick). The C++ engine itself needed zero changes for this.
- **A large dataset's `CsvSignalProvider` must be heap-allocated, not stack-allocated** — `CsvSignalProvider<float, 8, 10, 300000>` is ~20.6MB (exceeds macOS's default 8MB stack), so construct it via `std::make_unique`/`new`. This is legitimate: Provider construction is documented setup-phase work that happens before `NoHeapGuard` arms, exempt from the zero-allocation guarantee by design — only the hot-path `tick()` loop must stay allocation-free.

```sh
# Real dataset lives outside git (data/raw/ is gitignored — see the .gitignore
# entry) since it's a 5.48GB Zenodo archive. Download/unzip it yourself, then:
python3 tools/convert_epn612.py --inspect data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json
python3 tools/convert_epn612.py --convert data/raw/EMG-EPN612-Dataset/trainingJSON/user1/user1.json --out data/raw/epn612_user1.csv
```

`tests/test_real_data_integration.cpp` turns this into an automated, repeatable check rather than a one-off manual run: it processes the first 20,000 rows of `data/raw/epn612_user1.csv` (bounded on purpose — a `CsvSignalProvider` at the full ~298,710-row scale is tens of MB, and passing one by value through a constructor call still momentarily lives in that constructor's stack frame even when the enclosing object is heap-allocated) through a real `EdgeNeuro<8, 10>` and asserts `malloc_count == 0` for the whole run. It skips gracefully (Catch2 `SKIP()`) if the converted CSV isn't present, so it doesn't break the build for anyone who hasn't downloaded the dataset. Classifier weights here are arbitrary/untrained — this test verifies the zero-allocation guarantee against real-world data statistics, not classification accuracy.

Building this test surfaced a real bug, in the test harness rather than the engine: reading `NoHeapGuard::count()` after other `REQUIRE(...)` statements (rather than immediately when the armed scope closes) intermittently attributed Catch2's own assertion-machinery allocations to the "hot loop," since `record_allocation()` counts every allocation regardless of `armed()` state and only *aborts* when armed — a later, unrelated allocation between the guard closing and the count being read just silently inflates the same shared counter. The root cause was reasoned out from the code, not caught live in a debugger: since `record_allocation()` always aborts on an armed hit and the process never crashed, none of the extra allocations could have happened while armed, which points at something running *after* the guard closes but *before* the count is read — the intervening `REQUIRE(ticks == ...)`. (A debugger session did confirm Catch2's own machinery allocates via our overridden `operator new` — but those particular captured backtraces were static-initialization-time registrations from program startup, unrelated to the specific failure, since the conditional breakpoint meant to isolate armed-only hits didn't actually filter as intended.) Fixed by snapshotting the count into a local variable the instant the guarded block closes, before any other statement can run, and confirmed by 17+ consecutive clean runs (including fresh rebuilds, which is when the failure had shown up) afterward. All `NoHeapGuard`-checking tests follow this pattern now.

// Google Benchmark suite validating PRD section 5's performance gate:
// <32,0> continuous per-sample latency < 0.1ms, and malloc_count == 0 for
// every mode, using the same NoHeapGuard counter the unit tests use.

#include <benchmark/benchmark.h>

#include "edgeneuro/bench/latency_configs.hpp"
#include "edgeneuro/no_heap_guard.hpp"

// The two configurations live in edgeneuro/bench/latency_configs.hpp (2026-10-08), shared with the on-target
// cycle-count benchmark firmware/src/pipeline_latency_main.cpp so both measure exactly the same engines.
using namespace edgeneuro;
using edgeneuro::bench::HdEngine;
using edgeneuro::bench::make_hd_engine;
using edgeneuro::bench::make_wearable_engine;
using edgeneuro::bench::WearableEngine;

namespace {

// Mode A: <1, 6> wearable fusion -- 1ch EMG + 6-axis IMU.
void BM_WearableFusionTick(benchmark::State& state) {
    WearableEngine engine = make_wearable_engine();
    std::size_t out_class = 0;

    NoHeapGuard::reset_count();
    for (auto _ : state) {
        NoHeapGuard guard;
        engine.tick(out_class); // mutates `engine` and `out_class`; never a no-op the compiler can elide
        benchmark::DoNotOptimize(out_class);
    }
    state.counters["malloc_count"] = static_cast<double>(NoHeapGuard::count());
}
BENCHMARK(BM_WearableFusionTick);

// Mode B: <32, 0> high-density sEMG stress benchmark.
void BM_HighDensitySemgTick(benchmark::State& state) {
    HdEngine engine = make_hd_engine();
    std::size_t out_class = 0;

    NoHeapGuard::reset_count();
    for (auto _ : state) {
        NoHeapGuard guard;
        engine.tick(out_class); // mutates `engine` and `out_class`; never a no-op the compiler can elide
        benchmark::DoNotOptimize(out_class);
    }
    state.counters["malloc_count"] = static_cast<double>(NoHeapGuard::count());
}
BENCHMARK(BM_HighDensitySemgTick);

} // namespace

BENCHMARK_MAIN();

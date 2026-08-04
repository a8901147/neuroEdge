// Google Benchmark suite validating PRD section 5's performance gate:
// <32,0> continuous per-sample latency < 0.1ms, and malloc_count == 0 for
// every mode, using the same NoHeapGuard counter the unit tests use.

#include <array>
#include <benchmark/benchmark.h>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/no_heap_guard.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/sample.hpp"

using namespace edgeneuro;

namespace {

// In-memory provider: recycles a fixed buffer of synthetic samples so the
// benchmark measures steady-state DSP cost, not file I/O.
template <std::size_t EmgChannels, std::size_t ImuChannels, std::size_t N>
struct SyntheticProvider {
    using SampleT = Sample<float, EmgChannels, ImuChannels>;
    std::array<SampleT, N> samples{};
    std::size_t idx{0};

    SyntheticProvider() {
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t c = 0; c < EmgChannels; ++c) {
                samples[i].emg[c] = static_cast<float>((i + c) % 100) * 0.01f;
            }
            for (std::size_t c = 0; c < ImuChannels; ++c) {
                samples[i].imu[c] = static_cast<float>((i + c) % 50) * 0.02f;
            }
        }
    }

    bool next(SampleT& out) noexcept {
        out = samples[idx];
        idx = (idx + 1) % N; // wrap forever: benchmark controls iteration count
        return true;
    }
};

// -----------------------------------------------------------------------
// Mode A: <1, 6> wearable fusion — 1ch EMG + 6-axis IMU.
// -----------------------------------------------------------------------
constexpr std::size_t kWearableWindow = 50;
using WearableProvider = SyntheticProvider<1, 6, 256>;
using WearableClassifier = LdaClassifier<float, 7, 3>;
using WearableEngine = EdgeNeuro<
    float, 1, 6, kWearableWindow, WearableProvider,
    IirFilter<float>, PassThroughFilter<float>,
    MavFeature<float, kWearableWindow>, WearableClassifier>;

WearableEngine make_wearable_engine() {
    std::array<IirFilter<float>, 1> emg_filters{IirFilter<float>(0.2f, 0.0f, -0.2f, -0.6f, 0.2f)};
    std::array<PassThroughFilter<float>, 6> imu_filters{};
    std::array<std::array<float, 7>, 3> weights{};
    for (auto& row : weights) row.fill(0.1f);
    std::array<float, 3> bias{0.0f, 0.0f, 0.0f};
    return WearableEngine(
        WearableProvider{}, emg_filters, imu_filters,
        MavFeature<float, kWearableWindow>{}, WearableClassifier(weights, bias));
}

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

// -----------------------------------------------------------------------
// Mode B: <32, 0> high-density sEMG stress benchmark.
// -----------------------------------------------------------------------
constexpr std::size_t kHdWindow = 50;
using HdProvider = SyntheticProvider<32, 0, 256>;
using HdClassifier = LdaClassifier<float, 32, 4>;
using HdEngine = EdgeNeuro<
    float, 32, 0, kHdWindow, HdProvider,
    IirFilter<float>, PassThroughFilter<float>,
    MavFeature<float, kHdWindow>, HdClassifier>;

HdEngine make_hd_engine() {
    std::array<IirFilter<float>, 32> emg_filters{};
    emg_filters.fill(IirFilter<float>(0.2f, 0.0f, -0.2f, -0.6f, 0.2f));
    std::array<PassThroughFilter<float>, 0> imu_filters{};
    std::array<std::array<float, 32>, 4> weights{};
    for (auto& row : weights) row.fill(0.03f);
    std::array<float, 4> bias{0.0f, 0.0f, 0.0f, 0.0f};
    return HdEngine(
        HdProvider{}, emg_filters, imu_filters,
        MavFeature<float, kHdWindow>{}, HdClassifier(weights, bias));
}

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

#pragma once

// The two pipeline configurations whose per-sample latency is measured, shared so the host and the target measure
// exactly the same engines (2026-10-08):
//   benchmarks/bench_pipeline.cpp       -- Google Benchmark on the host (ns per tick)
//   firmware/src/pipeline_latency_main.cpp -- DWT cycle counter on the STM32F401 (cycles per tick)
// Moved here unchanged from bench_pipeline.cpp. Inputs are SYNTHETIC (a fixed repeating pattern), not recorded EMG.
//   <1,6>  wearable fusion: 1 sEMG channel + 6 IMU axes, IIR on the EMG, MAV over 50 samples, LDA into 3 classes
//   <32,0> high-density sEMG stress test: 32 sEMG channels, IIR each, MAV over 50 samples, LDA into 4 classes
// Header-only and allocation-free, so it builds for both the host and the bare-metal firmware.

#include <array>
#include <cstddef>
#include <cstdint>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/sample.hpp"

namespace edgeneuro::bench {

// In-memory provider: recycles a fixed buffer of synthetic samples so the measurement is the steady-state DSP cost,
// not file I/O.
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
        idx = (idx + 1) % N;   // wrap forever: the caller controls the iteration count
        return true;
    }
};

constexpr std::size_t kWindow = 50;

// How many synthetic samples each provider recycles. 256 on the host. The STM32F401 has 64 KB of SRAM and the <32,0>
// engine's buffer alone would be 32 KB (plus a temporary of the same size while it is constructed), so the firmware
// builds with EDGENEURO_BENCH_SAMPLES=64. The per-tick work is the same either way: next() copies one sample and wraps
// a power-of-two index.
#ifndef EDGENEURO_BENCH_SAMPLES
#define EDGENEURO_BENCH_SAMPLES 256
#endif
constexpr std::size_t kSamples = EDGENEURO_BENCH_SAMPLES;
static_assert((kSamples & (kSamples - 1u)) == 0u, "keep the wrap a power of two so its cost matches the host's");

// ---- <1,6> wearable fusion ----
constexpr std::size_t kWearableEmgChannels = 1;
constexpr std::size_t kWearableImuChannels = 6;
using WearableProvider = SyntheticProvider<kWearableEmgChannels, kWearableImuChannels, kSamples>;
using WearableClassifier = LdaClassifier<float, kWearableEmgChannels + kWearableImuChannels, 3>;
using WearableEngine = EdgeNeuro<float, kWearableEmgChannels, kWearableImuChannels, kWindow, WearableProvider,
                                 IirFilter<float>, PassThroughFilter<float>, MavFeature<float, kWindow>,
                                 WearableClassifier>;

inline WearableEngine make_wearable_engine() {
    std::array<IirFilter<float>, kWearableEmgChannels> emg_filters{IirFilter<float>(0.2f, 0.0f, -0.2f, -0.6f, 0.2f)};
    std::array<PassThroughFilter<float>, kWearableImuChannels> imu_filters{};
    std::array<std::array<float, kWearableEmgChannels + kWearableImuChannels>, 3> weights{};
    for (auto& row : weights) row.fill(0.1f);
    std::array<float, 3> bias{0.0f, 0.0f, 0.0f};
    return WearableEngine(WearableProvider{}, emg_filters, imu_filters, MavFeature<float, kWindow>{},
                          WearableClassifier(weights, bias));
}

// ---- <32,0> high-density sEMG stress test ----
constexpr std::size_t kHdEmgChannels = 32;
constexpr std::size_t kHdImuChannels = 0;
using HdProvider = SyntheticProvider<kHdEmgChannels, kHdImuChannels, kSamples>;
using HdClassifier = LdaClassifier<float, kHdEmgChannels, 4>;
using HdEngine = EdgeNeuro<float, kHdEmgChannels, kHdImuChannels, kWindow, HdProvider, IirFilter<float>,
                           PassThroughFilter<float>, MavFeature<float, kWindow>, HdClassifier>;

inline HdEngine make_hd_engine() {
    std::array<IirFilter<float>, kHdEmgChannels> emg_filters{};
    emg_filters.fill(IirFilter<float>(0.2f, 0.0f, -0.2f, -0.6f, 0.2f));
    std::array<PassThroughFilter<float>, kHdImuChannels> imu_filters{};
    std::array<std::array<float, kHdEmgChannels>, 4> weights{};
    for (auto& row : weights) row.fill(0.03f);
    std::array<float, 4> bias{0.0f, 0.0f, 0.0f, 0.0f};
    return HdEngine(HdProvider{}, emg_filters, imu_filters, MavFeature<float, kWindow>{}, HdClassifier(weights, bias));
}

// Min / max / mean of per-tick cycle counts. The sum is 64-bit, so a long run cannot overflow.
class CycleStats {
public:
    void add(std::uint32_t cycles) noexcept {
        if (n_ == 0u || cycles < min_) min_ = cycles;
        if (cycles > max_) max_ = cycles;
        sum_ += cycles;
        ++n_;
    }
    std::uint32_t count() const noexcept { return n_; }
    std::uint32_t min() const noexcept { return min_; }
    std::uint32_t max() const noexcept { return max_; }
    double mean() const noexcept { return n_ ? static_cast<double>(sum_) / static_cast<double>(n_) : 0.0; }

private:
    std::uint32_t n_ = 0u, min_ = 0u, max_ = 0u;
    std::uint64_t sum_ = 0u;
};

}  // namespace edgeneuro::bench

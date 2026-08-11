// End-to-end integration tests: wire the *real* CsvSignalProvider up to a
// full EdgeNeuro pipeline (real filters, real feature extraction, real
// classifier) and replay the actual synthetic fixture files in data/,
// exercising both PRD Architecture modes:
//   - EdgeNeuro<1,6>  : wearable prosthetic fusion (1x EMG + 6-axis IMU)
//   - EdgeNeuro<32,0> : high-density sEMG stress-test mode
//
// Unlike test_pipeline.cpp (which isolates pipeline scheduling logic with a
// mock in-memory provider), this file validates that every concept
// implementation composes correctly end-to-end against on-disk data, and
// that the fully-assembled pipeline still hits malloc_count == 0 for the
// entire run once construction (file I/O) is done.

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/providers/csv_signal_provider.hpp"

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
#include "edgeneuro/no_heap_guard.hpp"
#endif

using namespace edgeneuro;

namespace {
const std::string kDataDir = EDGENEURO_DATA_DIR;
}

TEST_CASE("EdgeNeuro<1,6> replays the wearable fusion fixture end-to-end", "[integration]") {
    constexpr std::size_t kWindow = 50;
    constexpr std::size_t kMaxSamples = 4096;
    using Provider = CsvSignalProvider<float, 1, 6, kMaxSamples>;
    using Classifier = LdaClassifier<float, 7, 2>;
    using Engine = EdgeNeuro<
        float, 1, 6, kWindow, Provider,
        IirFilter<float>, PassThroughFilter<float>,
        MavFeature<float, kWindow>, Classifier>;

    Provider provider(kDataDir + "/wearable_1emg_6imu.csv");
    REQUIRE(provider.size() == 2000); // matches tools/generate_sample_data.py's default row count

    std::array<IirFilter<float>, 1> emg_filters{IirFilter<float>(0.8f, -1.6f, 0.8f, -1.56f, 0.64f)};
    std::array<PassThroughFilter<float>, 6> imu_filters{};
    const std::array<std::array<float, 7>, 2> weights{{
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    }};
    const std::array<float, 2> bias{2.0f, 0.0f};

    Engine engine(std::move(provider), emg_filters, imu_filters, MavFeature<float, kWindow>{}, Classifier(weights, bias));

    std::size_t out_class = 0;
    std::size_t ticks = 0;
    std::size_t rest_count = 0;
    std::size_t grasp_count = 0;

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    NoHeapGuard::reset_count();
#endif
    {
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
        NoHeapGuard guard;
#endif
        while (engine.tick(out_class)) {
            ++ticks;
            if (engine.has_result()) {
                (out_class == 1 ? grasp_count : rest_count)++;
            }
        }
    }
    // Snapshot the count the instant the armed scope closes -- any statement
    // between here and reading it (including a REQUIRE, which can itself
    // allocate while decomposing its expression) would otherwise get
    // silently folded into "hot loop" allocations by the next read of a
    // shared counter that was never reset in between.
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    const std::size_t malloc_count = NoHeapGuard::count();
#endif

    REQUIRE(ticks == 2000);
    REQUIRE(ticks / kWindow == rest_count + grasp_count);
    // The synthetic generator alternates rest/contraction every ~0.8s, so a
    // 2s run must observe both classes at least once — otherwise the whole
    // filter -> feature -> classifier chain would be silently degenerate.
    REQUIRE(rest_count > 0);
    REQUIRE(grasp_count > 0);
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    REQUIRE(malloc_count == 0);
#endif
}

TEST_CASE("EdgeNeuro<32,0> replays the HD-sEMG stress fixture end-to-end", "[integration]") {
    constexpr std::size_t kWindow = 50;
    constexpr std::size_t kMaxSamples = 4096;
    using Provider = CsvSignalProvider<float, 32, 0, kMaxSamples>;
    using Classifier = LdaClassifier<float, 32, 2>;
    using Engine = EdgeNeuro<
        float, 32, 0, kWindow, Provider,
        IirFilter<float>, PassThroughFilter<float>,
        MavFeature<float, kWindow>, Classifier>;

    Provider provider(kDataDir + "/hd_semg_32ch.csv");
    REQUIRE(provider.size() == 2000);

    std::array<IirFilter<float>, 32> emg_filters{};
    emg_filters.fill(IirFilter<float>(0.8f, -1.6f, 0.8f, -1.56f, 0.64f));
    std::array<PassThroughFilter<float>, 0> imu_filters{};
    std::array<std::array<float, 32>, 2> weights{};
    weights[1].fill(1.0f); // class 1 fires when overall HD-sEMG energy is elevated
    const std::array<float, 2> bias{5.0f, 0.0f};

    Engine engine(std::move(provider), emg_filters, imu_filters, MavFeature<float, kWindow>{}, Classifier(weights, bias));

    std::size_t out_class = 0;
    std::size_t ticks = 0;
    std::size_t results = 0;

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    NoHeapGuard::reset_count();
#endif
    {
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
        NoHeapGuard guard;
#endif
        while (engine.tick(out_class)) {
            ++ticks;
            if (engine.has_result()) ++results;
        }
    }
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    const std::size_t malloc_count = NoHeapGuard::count();
#endif

    REQUIRE(ticks == 2000);
    REQUIRE(results == ticks / kWindow);
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    REQUIRE(malloc_count == 0);
#endif
}

#include <array>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/sample.hpp"

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
#include "edgeneuro/no_heap_guard.hpp"
#endif

using edgeneuro::EdgeNeuro;
using edgeneuro::LdaClassifier;
using edgeneuro::MavFeature;
using edgeneuro::PassThroughFilter;
using edgeneuro::Sample;

namespace {

// Deterministic mock provider satisfying edgeneuro::SignalProvider: replays
// a fixed in-memory sequence, no file I/O, so pipeline logic can be tested
// in isolation from CsvSignalProvider.
template <std::size_t EmgChannels, std::size_t ImuChannels, std::size_t N>
struct FixedProvider {
    using SampleT = Sample<float, EmgChannels, ImuChannels>;
    std::array<SampleT, N> samples{};
    std::size_t idx{0};

    bool next(SampleT& out) noexcept {
        if (idx >= N) return false;
        out = samples[idx++];
        return true;
    }
};

// EMG=1, IMU=2, WindowSize=2 => 3 features, 2 classes.
using Provider = FixedProvider<1, 2, 4>;
using Classifier = LdaClassifier<float, 3, 2>;
using Engine = EdgeNeuro<
    float, 1, 2, 2, Provider,
    PassThroughFilter<float>, PassThroughFilter<float>,
    MavFeature<float, 2>, Classifier>;

Engine make_engine(Provider provider) {
    std::array<PassThroughFilter<float>, 1> emg_filters{};
    std::array<PassThroughFilter<float>, 2> imu_filters{};
    // Class 1 fires when the combined MAV is large; class 0 otherwise.
    const std::array<std::array<float, 3>, 2> weights{{ {0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f} }};
    const std::array<float, 2> bias{1.0f, 0.0f};
    return Engine(std::move(provider), emg_filters, imu_filters, MavFeature<float, 2>{}, Classifier(weights, bias));
}

} // namespace

TEST_CASE("EdgeNeuro only produces a classification once a window is full", "[pipeline]") {
    Provider provider;
    provider.samples[0] = {{0.1f}, {0.1f, 0.1f}};
    provider.samples[1] = {{0.1f}, {0.1f, 0.1f}};
    provider.samples[2] = {{0.1f}, {0.1f, 0.1f}};
    provider.samples[3] = {{0.1f}, {0.1f, 0.1f}};

    Engine engine = make_engine(provider);
    std::size_t out_class = 999;

    REQUIRE(engine.tick(out_class));
    REQUIRE_FALSE(engine.has_result()); // window not full yet (WindowSize=2)

    REQUIRE(engine.tick(out_class));
    REQUIRE(engine.has_result()); // second sample completes the window
    REQUIRE(out_class == 0);      // small MAV => class 0 wins on bias
}

TEST_CASE("EdgeNeuro classifies large-amplitude windows into class 1", "[pipeline]") {
    Provider provider;
    provider.samples[0] = {{10.0f}, {10.0f, 10.0f}};
    provider.samples[1] = {{10.0f}, {10.0f, 10.0f}};

    Engine engine = make_engine(provider);
    std::size_t out_class = 999;
    engine.tick(out_class);
    REQUIRE(engine.tick(out_class));
    REQUIRE(engine.has_result());
    REQUIRE(out_class == 1);
}

TEST_CASE("EdgeNeuro::tick returns false once the provider is exhausted", "[pipeline]") {
    Provider provider; // all-zero samples, only 4 slots, but we drain all 4
    Engine engine = make_engine(provider);
    std::size_t out_class = 0;
    for (int i = 0; i < 4; ++i) {
        REQUIRE(engine.tick(out_class));
    }
    REQUIRE_FALSE(engine.tick(out_class));
}

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
TEST_CASE("EdgeNeuro::tick performs zero heap allocations in steady state", "[pipeline][no_heap]") {
    Provider provider;
    for (auto& s : provider.samples) {
        s = {{1.0f}, {1.0f, 1.0f}};
    }
    Engine engine = make_engine(provider);
    std::size_t out_class = 0;

    edgeneuro::NoHeapGuard::reset_count();
    {
        edgeneuro::NoHeapGuard guard;
        for (int i = 0; i < 4; ++i) {
            engine.tick(out_class);
        }
    }
    REQUIRE(edgeneuro::NoHeapGuard::count() == 0);
}
#endif

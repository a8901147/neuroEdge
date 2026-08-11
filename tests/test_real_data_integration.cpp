// Local-only integration test: proves malloc_count == 0 holds when the
// pipeline processes a REAL hardware dataset (EMG-EPN-612 Myo armband
// recordings), not just synthetic fixtures. Skips gracefully if the
// converted CSV isn't present locally -- the raw dataset is a 5.48GB
// Zenodo archive that can't (and shouldn't) be committed to the repo, so
// this only runs after you've downloaded + converted it yourself. See
// README "Real-dataset compatibility (EMG-EPN-612)".
//
// This intentionally does NOT check classification accuracy: the
// classifier below has arbitrary, untrained weights. The only thing being
// verified is the zero-allocation guarantee against real-world data
// statistics (real analog EMG noise, real IMU jitter) -- a different,
// narrower claim than "the demo decodes gestures correctly."
//
// Only reads the first kMaxSamples rows of the real file (CsvSignalProvider's
// documented capacity-limiting behavior, already covered by
// test_csv_provider.cpp), not the whole ~298K-row recording: a
// CsvSignalProvider this size is tens of MB, and passing one by value
// through a constructor call still materializes it in that constructor's
// stack frame even when the enclosing object is heap-allocated -- the same
// stack-overflow class of bug documented in README's "must be heap-allocated"
// note, just one level removed. Capping the row count keeps this test
// unconditionally safe regardless of how the compiler happens to handle
// the by-value Provider parameter, while still being real, unmodified
// hardware data -- the zero-alloc guarantee doesn't get any more or less
// true with more rows once it holds for one tick.

#ifdef EDGENEURO_HEAP_GUARD_ENABLED

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <memory>
#include <string>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/no_heap_guard.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/providers/csv_signal_provider.hpp"

using namespace edgeneuro;

namespace {
const std::string kRealDataPath = std::string(EDGENEURO_DATA_DIR) + "/raw/epn612_user1.csv";
}

TEST_CASE("EdgeNeuro<8,10> holds malloc_count == 0 against real EMG-EPN-612 data", "[integration][real_data]") {
    if (!std::filesystem::exists(kRealDataPath)) {
        SKIP("Real dataset not found at " << kRealDataPath
             << " -- see README 'Real-dataset compatibility' to download + convert it locally.");
    }

    constexpr std::size_t kEmgChannels = 8;
    constexpr std::size_t kImuChannels = 10; // accelerometer(3) + gyroscope(3) + quaternion(4)
    constexpr std::size_t kWindow = 50;
    constexpr std::size_t kMaxSamples = 20000; // bounded prefix of the real ~298,710-row file; see file header
    constexpr std::size_t kNumFeatures = kEmgChannels + kImuChannels;
    constexpr std::size_t kNumClasses = 2;

    using Provider = CsvSignalProvider<float, kEmgChannels, kImuChannels, kMaxSamples>;
    using Classifier = LdaClassifier<float, kNumFeatures, kNumClasses>;
    using Engine = EdgeNeuro<
        float, kEmgChannels, kImuChannels, kWindow, Provider,
        PassThroughFilter<float>, PassThroughFilter<float>,
        MavFeature<float, kWindow>, Classifier>;

    std::array<PassThroughFilter<float>, kEmgChannels> emg_filters{};
    std::array<PassThroughFilter<float>, kImuChannels> imu_filters{};
    std::array<std::array<float, kNumFeatures>, kNumClasses> weights{}; // arbitrary/untrained: accuracy not under test
    const std::array<float, kNumClasses> bias{0.0f, 0.0f};

    auto engine = std::make_unique<Engine>(
        Provider(kRealDataPath), emg_filters, imu_filters, MavFeature<float, kWindow>{}, Classifier(weights, bias));

    std::size_t out_class = 0;
    std::size_t ticks = 0;

    NoHeapGuard::reset_count();
    {
        NoHeapGuard guard;
        while (engine->tick(out_class)) {
            ++ticks;
        }
    }
    // Snapshot immediately -- see test_integration.cpp for why: any
    // statement between the guard closing and reading the counter (a
    // REQUIRE included, since decomposing its expression can itself
    // allocate) gets misattributed to "hot loop" allocations, which is
    // exactly what produced the intermittent false failures (count == 14)
    // this test showed before this fix, traced with a debugger to Catch2's
    // own assertion machinery running after the armed scope had already
    // closed but before the counter was read.
    const std::size_t malloc_count = NoHeapGuard::count();

    REQUIRE(ticks == kMaxSamples);
    REQUIRE(malloc_count == 0);
}

#endif // EDGENEURO_HEAP_GUARD_ENABLED

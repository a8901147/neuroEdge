// Phase 1 turnkey demo: a terminal oscilloscope for the <1,6> wearable
// fusion mode (PRD 4.3 "Turnkey Scientific Oscilloscope"). Deliberately
// avoids any GUI/web toolkit — this loop is the entire self-check surface
// for V1.0: live EMG amplitude, decoded gesture, per-tick latency, and a
// running malloc_count that must stay at zero for the whole run.
//
// NoHeapGuard is armed ONLY around engine.tick() — the DSP hot path. The
// terminal rendering below it uses buffered stdio/iostream, which is
// legitimate Host-side demo scaffolding, not part of the real-time
// guarantee this project is validating.

#include <array>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

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

constexpr std::size_t kEmgChannels = 1;
constexpr std::size_t kImuChannels = 6;
constexpr std::size_t kWindowSize = 50;   // 50ms window @ 1kHz
constexpr std::size_t kMaxSamples = 8192; // covers the generated fixture (2000 rows) with headroom
constexpr std::size_t kNumFeatures = kEmgChannels + kImuChannels;
constexpr std::size_t kNumClasses = 2;    // 0 = rest, 1 = grasp

using Provider = CsvSignalProvider<float, kEmgChannels, kImuChannels, kMaxSamples>;
using Classifier = LdaClassifier<float, kNumFeatures, kNumClasses>;
using Engine = EdgeNeuro<
    float, kEmgChannels, kImuChannels, kWindowSize, Provider,
    IirFilter<float>, PassThroughFilter<float>,
    MavFeature<float, kWindowSize>, Classifier>;

Engine build_engine(const std::string& csv_path) {
    // 2nd-order Butterworth-shaped high-pass-leaning biquad: suppresses DC
    // drift/motion artifact while passing EMG-band energy. Precomputed
    // offline, loaded as a static constant per PRD 4.1.
    std::array<IirFilter<float>, kEmgChannels> emg_filters{IirFilter<float>(0.8f, -1.6f, 0.8f, -1.56f, 0.64f)};
    std::array<PassThroughFilter<float>, kImuChannels> imu_filters{};

    // Feature order is [emg_mav, imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz].
    // Only the EMG MAV term drives the rest/grasp decision for this demo;
    // IMU features carry zero weight here but remain part of the fused
    // feature vector, ready for a richer decoder later.
    const std::array<std::array<float, kNumFeatures>, kNumClasses> weights{{
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    }};
    const std::array<float, kNumClasses> bias{2.0f, 0.0f}; // rest wins unless EMG MAV clearly elevated

    return Engine(Provider(csv_path), emg_filters, imu_filters, MavFeature<float, kWindowSize>{}, Classifier(weights, bias));
}

void render(float emg_mav, std::size_t gesture_class, double last_tick_us, double avg_tick_us,
            double max_tick_us, std::size_t malloc_count, std::size_t sample_index) {
    constexpr int kBarWidth = 40;
    const int filled = std::min(kBarWidth, static_cast<int>(emg_mav * kBarWidth * 4.0f));

    std::printf("\x1b[H\x1b[2J"); // clear + home
    std::printf("EdgeNeuro Phase 1 -- Turnkey Scientific Oscilloscope\n");
    std::printf("Mode: EdgeNeuro<1,6> wearable fusion (1x EMG + 6-axis IMU @ 1kHz)\n\n");

    std::printf("EMG MAV  [");
    for (int i = 0; i < kBarWidth; ++i) std::putchar(i < filled ? '#' : '.');
    std::printf("] %.4f\n\n", static_cast<double>(emg_mav));

    std::printf("Decoded gesture : %s\n\n", gesture_class == 1 ? "GRASP" : "rest ");

    std::printf("Latency (tick)  : last=%.3f us   avg=%.3f us   max=%.3f us   [target < 100 us]\n",
                last_tick_us, avg_tick_us, max_tick_us);
    std::printf("malloc_count    : %zu   [must stay 0]\n", malloc_count);
    std::printf("samples played  : %zu\n", sample_index);
    std::printf("\nCtrl+C to exit.\n");
    std::fflush(stdout);
}

} // namespace

int main(int argc, char** argv) {
    const std::string csv_path = argc > 1 ? argv[1] : "data/wearable_1emg_6imu.csv";

    Engine engine = build_engine(csv_path);

    std::size_t out_class = 0;
    std::size_t sample_index = 0;
    double total_tick_us = 0.0;
    double max_tick_us = 0.0;

#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    NoHeapGuard::reset_count();
#endif

    while (true) {
        const auto start = std::chrono::steady_clock::now();
        bool ok;
        {
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
            NoHeapGuard guard; // armed only around the DSP hot path
#endif
            ok = engine.tick(out_class);
        }
        const auto end = std::chrono::steady_clock::now();
        if (!ok) break;

        const double tick_us = std::chrono::duration<double, std::micro>(end - start).count();
        ++sample_index;
        total_tick_us += tick_us;
        max_tick_us = std::max(max_tick_us, tick_us);

        // Pace to the 1kHz stream rate the PRD specifies (data was generated
        // at fs=1000Hz): sleep off whatever's left of this 1ms slot after the
        // (sub-microsecond) DSP tick. This is demo scaffolding only — it runs
        // after the timed/guarded hot-path region above, so it never pollutes
        // the latency measurement or the allocation count.
        std::this_thread::sleep_for(std::chrono::microseconds(1000) - std::chrono::duration_cast<std::chrono::microseconds>(end - start));

        if (sample_index % 20 == 0) {
            // Redraw at ~50Hz (every 20 samples of a 1kHz stream) instead of
            // every tick, to keep the terminal legible.
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
            const std::size_t malloc_count = NoHeapGuard::count();
#else
            const std::size_t malloc_count = 0;
#endif
            render(engine.last_features()[0], out_class, tick_us,
                   total_tick_us / static_cast<double>(sample_index), max_tick_us, malloc_count, sample_index);
        }
    }

    std::printf("\nStream finished after %zu samples.\n", sample_index);
    std::printf("avg tick latency = %.3f us, max = %.3f us\n",
                sample_index ? total_tick_us / static_cast<double>(sample_index) : 0.0, max_tick_us);
#ifdef EDGENEURO_HEAP_GUARD_ENABLED
    std::printf("final malloc_count (hot loop) = %zu\n", NoHeapGuard::count());
#endif
    return 0;
}

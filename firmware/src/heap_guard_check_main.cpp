// Stage 2: answers Phase 1.5's unknown #1 -- does NoHeapGuard actually
// stay at malloc_count == 0 on real ARM GCC-compiled code running on the
// actual target, not just on Host? Same Engine/tick() shape as Stage 1's
// footprint_check_main.cpp, but now the loop runs inside an armed
// NoHeapGuard scope (matching the exact pattern used in
// tests/test_pipeline.cpp and tests/test_real_data_integration.cpp on
// Host), for many iterations.
//
// LED signals (distinguishable by eye, no UART needed yet):
//   slow blink forever  -> unreachable in this build (Stage 0 pattern, for reference)
//   solid ON forever    -> PASS: loop finished, zero allocations detected
//   fast blink forever  -> FAIL: NoHeapGuard caught a real allocation
//                          (signal_violation_and_halt() in no_heap_guard_target.cpp)

#include "stm32f4xx.h"

#include <array>
#include <cstddef>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/no_heap_guard.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/sample.hpp"

namespace {

constexpr std::size_t kEmgChannels = 1;
constexpr std::size_t kImuChannels = 6;
constexpr std::size_t kWindow = 50;
constexpr std::size_t kNumFeatures = kEmgChannels + kImuChannels;
constexpr std::size_t kNumClasses = 2;
constexpr unsigned kLedPin = 13u;

// Same stand-in as Stage 1 -- see footprint_check_main.cpp.
struct StubProvider {
    using SampleT = edgeneuro::Sample<float, kEmgChannels, kImuChannels>;
    bool next(SampleT& out) noexcept {
        out.emg.fill(0.5f);
        out.imu.fill(0.1f);
        return true;
    }
};

using Classifier = edgeneuro::LdaClassifier<float, kNumFeatures, kNumClasses>;
using Engine = edgeneuro::EdgeNeuro<
    float, kEmgChannels, kImuChannels, kWindow, StubProvider,
    edgeneuro::IirFilter<float>, edgeneuro::PassThroughFilter<float>,
    edgeneuro::MavFeature<float, kWindow>, Classifier>;

void led_solid_on() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (kLedPin * 2u));
    GPIOC->MODER |= (1u << (kLedPin * 2u));
    GPIOC->ODR &= ~(1u << kLedPin); // active-low: clear bit = LED on
}

} // namespace

int main(void) {
    std::array<edgeneuro::IirFilter<float>, kEmgChannels> emg_filters{
        edgeneuro::IirFilter<float>(0.8f, -1.6f, 0.8f, -1.56f, 0.64f)};
    std::array<edgeneuro::PassThroughFilter<float>, kImuChannels> imu_filters{};
    std::array<std::array<float, kNumFeatures>, kNumClasses> weights{};
    weights[1].fill(1.0f);
    const std::array<float, kNumClasses> bias{2.0f, 0.0f};

    Engine engine(StubProvider{}, emg_filters, imu_filters, edgeneuro::MavFeature<float, kWindow>{},
                  Classifier(weights, bias));

    std::size_t out_class = 0;

    edgeneuro::NoHeapGuard::reset_count();
    {
        edgeneuro::NoHeapGuard guard;
        // 100,000 ticks: comfortably longer than any single Host test run,
        // enough to catch a violation that only manifests after the
        // classifier's window-boundary branch (every kWindow-th tick) has
        // fired many times over.
        for (std::size_t i = 0; i < 100000u; ++i) {
            engine.tick(out_class);
        }
    }
    // Reaching here at all means the guarded scope closed without
    // signal_violation_and_halt() ever firing -- i.e. malloc_count stayed
    // at 0 for the entire run, verified on the real target, not Host.
    led_solid_on();
    while (true) {
    }
}

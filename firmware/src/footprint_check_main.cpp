// Stage 1: compiles the REAL Phase 1 engine (include/edgeneuro/) for
// STM32F401, unmodified -- proves the "zero modification" claim in
// PRD Phase 3 is actually true, not aspirational. No CsvSignalProvider
// (needs a filesystem the MCU doesn't have); a minimal stub Provider
// stands in, matching the shape the real Stm32AdcProvider will have.
//
// This binary doesn't need to run correctly to answer Stage 1's question
// (Flash/SRAM footprint, via `arm-none-eabi-size` on the .elf) -- but the
// tick() loop's result still drives the LED so the optimizer can't prove
// the engine's output is unused and strip it out, which would make the
// footprint measurement meaningless.

#include "stm32f4xx.h"

#include <array>
#include <cstddef>

#include "edgeneuro/classifiers/lda_classifier.hpp"
#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"
#include "edgeneuro/pipeline.hpp"
#include "edgeneuro/sample.hpp"

namespace {

constexpr std::size_t kEmgChannels = 1;
constexpr std::size_t kImuChannels = 6;
constexpr std::size_t kWindow = 50;
constexpr std::size_t kNumFeatures = kEmgChannels + kImuChannels;
constexpr std::size_t kNumClasses = 2;

// Stand-in for Stm32AdcProvider (not written yet -- that's full Phase 3).
// Satisfies the same SignalProvider concept the real ADC/DMA-backed
// provider will, so swapping this out later touches nothing downstream.
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

constexpr unsigned kLedPin = 13u;

void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

} // namespace

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (kLedPin * 2u));
    GPIOC->MODER |= (1u << (kLedPin * 2u));

    std::array<edgeneuro::IirFilter<float>, kEmgChannels> emg_filters{
        edgeneuro::IirFilter<float>(0.8f, -1.6f, 0.8f, -1.56f, 0.64f)};
    std::array<edgeneuro::PassThroughFilter<float>, kImuChannels> imu_filters{};
    std::array<std::array<float, kNumFeatures>, kNumClasses> weights{};
    weights[1].fill(1.0f);
    const std::array<float, kNumClasses> bias{2.0f, 0.0f};

    Engine engine(StubProvider{}, emg_filters, imu_filters, edgeneuro::MavFeature<float, kWindow>{},
                  Classifier(weights, bias));

    std::size_t out_class = 0;
    while (true) {
        engine.tick(out_class);
        if (engine.has_result() && out_class == 1) {
            GPIOC->ODR ^= (1u << kLedPin); // observable sink for the engine's output
        }
        delay(80000u);
    }
}

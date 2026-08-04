#pragma once

#include <array>
#include <concepts>
#include <cstddef>

#include "edgeneuro/sample.hpp"

namespace edgeneuro {

// Signal Provider: pushes one synchronized Sample per call. Returns false
// when no new sample is available (Host CSV EOF) or on stream fault.
// Host (CsvSignalProvider) and Target (Stm32AdcProvider, Phase 3) implement
// the exact same contract, so Pipeline Core never changes.
template <typename T, typename ValueType, std::size_t EmgChannels, std::size_t ImuChannels>
concept SignalProvider = requires(T provider, Sample<ValueType, EmgChannels, ImuChannels>& sample) {
    { provider.next(sample) } -> std::same_as<bool>;
};

// Filter: a per-channel streaming transform. Must be default-constructible
// state machines (IIR delay lines, etc.) that can be reset without reallocating.
template <typename T, typename ValueType>
concept Filter = requires(T filter, ValueType value) {
    { filter.process(value) } -> std::same_as<ValueType>;
    { filter.reset() } -> std::same_as<void>;
};

// Feature: reduces a fixed-size window of one channel's filtered samples
// down to a single scalar (MAV, RMS, ...).
template <typename T, typename ValueType, std::size_t WindowSize>
concept Feature = requires(T feature, const std::array<ValueType, WindowSize>& window) {
    { feature.compute(window) } -> std::same_as<ValueType>;
};

// Classifier: maps a fixed-size feature vector (one scalar per channel) to
// a discrete class index.
template <typename T, typename ValueType, std::size_t NumFeatures>
concept Classifier = requires(T classifier, const std::array<ValueType, NumFeatures>& features) {
    { classifier.classify(features) } -> std::same_as<std::size_t>;
};

} // namespace edgeneuro

#pragma once

#include <array>
#include <cstddef>

#include "edgeneuro/concepts.hpp"
#include "edgeneuro/sample.hpp"

namespace edgeneuro {

// EdgeNeuro: the compile-time-wired Phase 1 pipeline.
//
//   EdgeNeuro<1, 6, float, MyoBandpass, PassThroughFilter<float>,
//             MavFeature<float, 200>, LdaClassifier<float, 7, 3>>
//
// swaps algorithms by changing one `using` line; the Ring Buffer and
// scheduling logic underneath never move. EMG and IMU channels get
// independent filter strategies (EmgFilterT / ImuFilterT) since they have
// unrelated spectral characteristics — EMG needs a 20-450Hz-class bandpass,
// IMU typically wants light smoothing or none at all.
//
// tick() is the only method called from the hot loop: it pulls one sample
// from the Provider, filters every channel, accumulates into a fixed
// WindowSize buffer per channel, and once a window is full, extracts one
// feature per channel and classifies. Every buffer is a std::array sized
// at compile time — there is no allocation anywhere in this class.
template <
    typename ValueType,
    std::size_t EmgChannels,
    std::size_t ImuChannels,
    std::size_t WindowSize,
    typename Provider,
    typename EmgFilterT,
    typename ImuFilterT,
    typename FeatureT,
    typename ClassifierT>
    requires SignalProvider<Provider, ValueType, EmgChannels, ImuChannels> &&
             Filter<EmgFilterT, ValueType> &&
             Filter<ImuFilterT, ValueType> &&
             Feature<FeatureT, ValueType, WindowSize> &&
             Classifier<ClassifierT, ValueType, EmgChannels + ImuChannels>
class EdgeNeuro {
public:
    static constexpr std::size_t TotalChannels = EmgChannels + ImuChannels;
    static constexpr std::size_t NumFeatures = TotalChannels;

    using SampleT = Sample<ValueType, EmgChannels, ImuChannels>;
    using WindowT = std::array<ValueType, WindowSize>;
    using FeatureVec = std::array<ValueType, NumFeatures>;

    EdgeNeuro(
        Provider provider,
        std::array<EmgFilterT, EmgChannels> emg_filters,
        std::array<ImuFilterT, ImuChannels> imu_filters,
        FeatureT feature,
        ClassifierT classifier) noexcept
        : provider_(std::move(provider)),
          emg_filters_(std::move(emg_filters)),
          imu_filters_(std::move(imu_filters)),
          feature_(std::move(feature)),
          classifier_(std::move(classifier)) {}

    // Advances the pipeline by exactly one sample. Returns false when the
    // Provider is exhausted (Host CSV EOF). `out_class` is only written
    // when has_result() would return true for this tick — check it before
    // reading a stale class index.
    bool tick(std::size_t& out_class) noexcept {
        SampleT sample;
        if (!provider_.next(sample)) {
            return false;
        }

        for (std::size_t c = 0; c < EmgChannels; ++c) {
            windows_[c][window_pos_] = emg_filters_[c].process(sample.emg[c]);
        }
        for (std::size_t c = 0; c < ImuChannels; ++c) {
            windows_[EmgChannels + c][window_pos_] = imu_filters_[c].process(sample.imu[c]);
        }

        ++window_pos_;
        has_result_ = false;
        if (window_pos_ < WindowSize) {
            return true;
        }
        window_pos_ = 0;

        for (std::size_t c = 0; c < TotalChannels; ++c) {
            last_features_[c] = feature_.compute(windows_[c]);
        }
        out_class = classifier_.classify(last_features_);
        has_result_ = true;
        return true;
    }

    bool has_result() const noexcept { return has_result_; }

    // Last computed feature vector (one scalar per channel, EMG channels
    // first). Exposed read-only for Host-side visualization/telemetry
    // (e.g. the terminal oscilloscope demo) — not used internally beyond
    // tick(), and never allocated: it is a fixed member of the pipeline.
    const FeatureVec& last_features() const noexcept { return last_features_; }

    void reset() noexcept {
        for (auto& f : emg_filters_) f.reset();
        for (auto& f : imu_filters_) f.reset();
        window_pos_ = 0;
        has_result_ = false;
    }

private:
    Provider provider_;
    std::array<EmgFilterT, EmgChannels> emg_filters_;
    std::array<ImuFilterT, ImuChannels> imu_filters_;
    FeatureT feature_;
    ClassifierT classifier_;

    std::array<WindowT, TotalChannels> windows_{};
    FeatureVec last_features_{};
    std::size_t window_pos_{0};
    bool has_result_{false};
};

} // namespace edgeneuro

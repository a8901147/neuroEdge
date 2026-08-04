#pragma once

#include <array>
#include <cstddef>

namespace edgeneuro {

// Linear Discriminant Analysis classifier: argmax over per-class linear
// scores W*x + b. Weights are trained offline and loaded once at
// construction (static, immutable for the lifetime of the pipeline) —
// no allocation, no re-fitting on the device.
template <typename ValueType, std::size_t NumFeatures, std::size_t NumClasses>
class LdaClassifier {
public:
    constexpr LdaClassifier(
        const std::array<std::array<ValueType, NumFeatures>, NumClasses>& weights,
        const std::array<ValueType, NumClasses>& bias) noexcept
        : weights_(weights), bias_(bias) {}

    std::size_t classify(const std::array<ValueType, NumFeatures>& features) const noexcept {
        std::size_t best_class = 0;
        ValueType best_score = score(0, features);
        for (std::size_t c = 1; c < NumClasses; ++c) {
            const ValueType s = score(c, features);
            if (s > best_score) {
                best_score = s;
                best_class = c;
            }
        }
        return best_class;
    }

private:
    ValueType score(std::size_t c, const std::array<ValueType, NumFeatures>& features) const noexcept {
        ValueType sum = bias_[c];
        for (std::size_t f = 0; f < NumFeatures; ++f) {
            sum += weights_[c][f] * features[f];
        }
        return sum;
    }

    std::array<std::array<ValueType, NumFeatures>, NumClasses> weights_;
    std::array<ValueType, NumClasses> bias_;
};

} // namespace edgeneuro

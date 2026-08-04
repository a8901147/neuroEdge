#pragma once

#include <array>
#include <cstddef>

namespace edgeneuro {

// Mean Absolute Value: classic EMG time-domain feature, proportional to
// muscle contraction intensity.
template <typename ValueType, std::size_t WindowSize>
class MavFeature {
public:
    ValueType compute(const std::array<ValueType, WindowSize>& window) const noexcept {
        ValueType sum{0};
        for (ValueType v : window) {
            sum += v < ValueType{0} ? -v : v;
        }
        return sum / static_cast<ValueType>(WindowSize);
    }
};

} // namespace edgeneuro

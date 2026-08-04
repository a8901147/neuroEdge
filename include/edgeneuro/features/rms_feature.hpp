#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace edgeneuro {

// Root Mean Square: energy-based EMG time-domain feature, more sensitive
// to high-amplitude bursts than MAV.
template <typename ValueType, std::size_t WindowSize>
class RmsFeature {
public:
    ValueType compute(const std::array<ValueType, WindowSize>& window) const noexcept {
        ValueType sum{0};
        for (ValueType v : window) {
            sum += v * v;
        }
        using std::sqrt;
        return sqrt(sum / static_cast<ValueType>(WindowSize));
    }
};

} // namespace edgeneuro

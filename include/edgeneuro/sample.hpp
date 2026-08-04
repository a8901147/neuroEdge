#pragma once

#include <array>
#include <cstddef>

namespace edgeneuro {

// One synchronized time-step of sensor data: EmgChannels muscle channels
// plus ImuChannels inertial channels, sampled at the same instant.
template <typename ValueType, std::size_t EmgChannels, std::size_t ImuChannels>
struct Sample {
    std::array<ValueType, EmgChannels> emg{};
    std::array<ValueType, ImuChannels> imu{};
};

} // namespace edgeneuro

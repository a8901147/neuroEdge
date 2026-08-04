#pragma once

namespace edgeneuro {

// Identity filter: satisfies the Filter concept without altering the signal.
// Used as the baseline strategy for A/B comparison against IirFilter and
// as the default when no spatial filtering is needed (e.g. IMU channels).
template <typename ValueType>
class PassThroughFilter {
public:
    ValueType process(ValueType value) noexcept { return value; }
    void reset() noexcept {}
};

} // namespace edgeneuro

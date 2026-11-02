#pragma once

namespace edgeneuro {

// Caps how fast an output may change per second, so a step target (e.g. GripStateMachine's 0/1) becomes a ramp at
// the actuator. It shapes a setpoint, not a measurement. Not a Filter (concepts.hpp): a rate needs dt, which
// Filter::process(value) does not take.
template <typename ValueType>
class SlewRateLimiter {
public:
    // max_rate: largest allowed |change in output| per second. Must be
    // positive; a target further than max_rate * dt away is approached
    // gradually instead of jumped to.
    explicit SlewRateLimiter(ValueType max_rate, ValueType initial = ValueType{0}) noexcept
        : max_rate_(max_rate), current_(initial) {}

    // Advances current_ toward target by at most max_rate_ * dt (seconds)
    // this call, and returns the new current_. Never overshoots: if
    // target is already within one step, lands exactly on it rather than
    // oscillating past and back.
    ValueType update(ValueType target, ValueType dt) noexcept {
        const ValueType max_step = max_rate_ * dt;
        const ValueType delta = target - current_;
        if (delta > max_step) {
            current_ += max_step;
        } else if (delta < -max_step) {
            current_ -= max_step;
        } else {
            current_ = target;
        }
        return current_;
    }

    ValueType value() const noexcept { return current_; }

    void reset(ValueType value = ValueType{0}) noexcept { current_ = value; }

private:
    ValueType max_rate_;
    ValueType current_;
};

} // namespace edgeneuro

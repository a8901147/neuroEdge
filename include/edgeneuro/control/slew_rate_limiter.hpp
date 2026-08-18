#pragma once

namespace edgeneuro {

// Caps how fast an output value is allowed to change per second, so a
// discontinuous target (e.g. GripStateMachine's 0/1 step) turns into a
// smooth ramp instead of an instantaneous jump at the actuator.
//
// This is the "command trajectory smoothing" step that sits downstream of
// a decision, not a sensor-fusion step: ComplementaryFilter smooths a
// noisy *measurement* into a believable angle; this smooths a *setpoint*
// so the actuator's motion path is continuous. The two are independent --
// residual jitter on ComplementaryFilter's roll()/pitch() output doesn't
// need a new component at all, just the project's existing IirFilter
// (include/edgeneuro/filters/iir_filter.hpp) configured as a gentle
// low-pass, since that jitter is exactly the kind of per-sample noise
// IirFilter already exists to remove.
//
// Deliberately not a Filter (concepts.hpp): that concept's process(value)
// takes no dt, but slew rate is fundamentally a rate (units/second), so
// it needs the time step explicitly -- same reasoning that keeps
// ComplementaryFilter and GripStateMachine out of that concept too.
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

#pragma once

#include <cmath>

namespace edgeneuro {

// Per-servo output stage: starts at a rest pulse and walks toward the commanded
// pulse at a slow "start" rate until it has arrived, then follows at a fast "track"
// rate (latched -- it never falls back to the slow rate).
//
// What it can and cannot do: an open-loop hobby servo has no position feedback and
// does not return anywhere when unpowered, so the FIRST pulse after power-up moves
// it from wherever it happened to be, at the servo's own full speed -- this cannot
// slow that. It controls what comes after: the output does not jump from the rest
// pulse straight to wherever the sensors say the arm should be.
//
// Rest pulse and both rates are PLACEHOLDERS until measured on the real arm
// (SESSION_LOG 2026-09-26); this class only enforces the shape of the motion.
// Inputs are floats so a NaN target / time step is detectable and simply holds the
// current pulse. The internal position is a float: at 1 kHz a slow start rate
// moves well under 1 us per tick, so rounding the state to whole microseconds each
// tick would stall it completely.
class ServoStartupRamp {
public:
    ServoStartupRamp(unsigned rest_us, unsigned min_us, unsigned max_us, float start_rate_us_per_s,
                     float track_rate_us_per_s, float arrived_within_us) noexcept
        : min_(static_cast<float>(min_us)),
          max_(static_cast<float>(max_us)),
          start_rate_(start_rate_us_per_s),
          track_rate_(track_rate_us_per_s),
          tol_(arrived_within_us) {
        current_ = clamp(static_cast<float>(rest_us));
    }

    // The pulse width to output this tick, moving toward target_us.
    unsigned step(float target_us, float dt_s) noexcept {
        if (!std::isfinite(dt_s) || dt_s <= 0.0f || std::isnan(target_us)) {
            return current_us();
        }
        const float target = clamp(target_us);      // +-inf clamp to the range ends
        const float delta = target - current_;
        // the rate for THIS step is decided before the arrival check, so the step
        // that arrives is still bound by the slow rate
        const float max_step = (tracking_ ? track_rate_ : start_rate_) * dt_s;
        if (delta > max_step) {
            current_ += max_step;
        } else if (delta < -max_step) {
            current_ -= max_step;
        } else {
            current_ = target;
        }
        if (std::fabs(target - current_) <= tol_) {
            tracking_ = true;
        }
        return current_us();
    }

    // Back to the slow start-up rate from wherever the output is now (2026-10-03: the firmware's R command walks every
    // servo back to its start pose slowly, and then follows the arm again starting slowly).
    void rearm() noexcept { tracking_ = false; }

    unsigned current_us() const noexcept { return static_cast<unsigned>(current_ + 0.5f); }
    bool tracking() const noexcept { return tracking_; }

private:
    float clamp(float v) const noexcept { return v < min_ ? min_ : (v > max_ ? max_ : v); }

    float min_, max_, start_rate_, track_rate_, tol_;
    float current_ = 0.0f;
    bool tracking_ = false;
};

} // namespace edgeneuro

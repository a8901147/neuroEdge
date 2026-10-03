#pragma once

#include <cmath>

namespace edgeneuro {

// Linearly maps a live sensor-derived value (e.g. shoulder_pitch in
// radians, or a 0..1 grip scalar) onto a hobby servo's real, empirically
// measured pulse-width range, clamping to that range.
//
// Deliberately NOT inverse kinematics: driving a MEArm from live IMU/EMG
// signals maps each sensor value straight onto ONE servo (this project's
// 2026-09 SESSION_LOG entry has the reasoning) rather than converting to a
// Cartesian target and solving all 3 arm servos together -- the human
// joint angles and the MEArm's own joint angles aren't the same
// kinematic quantities to begin with (different pivot locations), so nothing
// is lost by mapping each one independently instead of adding an IK layer
// this project doesn't otherwise need.
//
// value_min/value_max is the REAL sensor range this axis is expected to
// produce (e.g. run_demo_live.py's SHOULDER_PITCH_RANGE, or 0.0f/1.0f for
// grip) -- must satisfy value_min < value_max. pulse_min_us/pulse_max_us
// is the servo's OWN real measured safe range for this joint, found by
// ear via servo_limit_finder_4ch once mounted on the assembled MEArm
// (see servo_pwm_test_main.c's 2026-09-21 comment for why this has to be
// measured per unit/per joint, not assumed from a datasheet). Which
// physical end of pulse_min_us/pulse_max_us corresponds to which end of
// value_min/value_max is the caller's choice (e.g. if a more-negative
// shoulder_pitch should swing the servo toward its LARGER pulse width,
// pass pulse_min_us/pulse_max_us swapped relative to the "small pulse =
// small pulse width number" reading) -- this class only assumes value_min
// < value_max, not any particular pulse-width polarity.
class ServoAngleMap {
public:
    // constexpr: the firmware keeps these in function-local statics, which must be constant-initialized (a run-time
    // initialized static needs __cxa_guard_acquire -> abort() -> no link on bare metal).
    constexpr ServoAngleMap(float value_min, float value_max,
                            unsigned pulse_min_us, unsigned pulse_max_us) noexcept
        : value_min_(value_min), value_max_(value_max),
          pulse_min_us_(pulse_min_us), pulse_max_us_(pulse_max_us) {}

    // Clamps value to [value_min, value_max], then linearly rescales it
    // onto [pulse_min_us, pulse_max_us]. Rounds to the nearest microsecond
    // (TIM3's CCRx registers this eventually feeds are integer ticks).
    unsigned pulse_us(float value) const noexcept {
        // Fail safe, never out of range: a NaN input (e.g. a filter fed a bad
        // sample) fails every comparison below, and an empty / reversed /
        // non-finite / overflowing value range makes the division below 0/0 or
        // inf/inf = NaN. Either would reach static_cast<unsigned>(NaN), which is
        // undefined behaviour -- on the chip possibly 0 or 0xFFFFFFFF, i.e. a
        // servo slammed into its stop. Command the pulse-range midpoint
        // (neutral) instead.
        const float span = value_max_ - value_min_;
        if (value != value || !(span > 0.0f) || !std::isfinite(span)) {
            return midpoint_us();
        }
        float clamped = value;
        if (clamped < value_min_) {
            clamped = value_min_;
        } else if (clamped > value_max_) {
            clamped = value_max_;
        }
        const float t = (clamped - value_min_) / (value_max_ - value_min_);
        // pulse_max_us_ - pulse_min_us_ must NOT be computed in unsigned
        // arithmetic: the reversed-polarity case (pulse_max_us_ <
        // pulse_min_us_, e.g. a larger value should command a SMALLER
        // pulse width) makes that subtraction underflow/wrap instead of
        // going negative. Cast each operand to float first, then
        // subtract, so a genuinely negative delta stays negative.
        const float pulse_min_f = static_cast<float>(pulse_min_us_);
        const float pulse_max_f = static_cast<float>(pulse_max_us_);
        const float pulse = pulse_min_f + t * (pulse_max_f - pulse_min_f);
        return static_cast<unsigned>(pulse + 0.5f);
    }

    constexpr unsigned pulse_min_us() const noexcept { return pulse_min_us_; }
    constexpr unsigned pulse_max_us() const noexcept { return pulse_max_us_; }

private:
    unsigned midpoint_us() const noexcept {
        return static_cast<unsigned>(
            (static_cast<float>(pulse_min_us_) + static_cast<float>(pulse_max_us_)) * 0.5f + 0.5f);
    }

    float value_min_;
    float value_max_;
    unsigned pulse_min_us_;
    unsigned pulse_max_us_;
};

} // namespace edgeneuro

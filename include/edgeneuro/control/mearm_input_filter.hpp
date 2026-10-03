#pragma once

#include <cmath>

#include "edgeneuro/control/mearm_pathb.hpp"
#include "edgeneuro/filters/vec3_one_euro.hpp"

namespace edgeneuro::mearm {

// Input conditioning for the real MEArm (2026-10-03). The servos run at their full speed -- the start-up ramp's rate is a
// safety limit, not a smoothing tool -- and the COMMAND is made steady instead, as teleoperation usually does:
//   1. a 1-euro filter on both arm IMUs' raw accel vectors (strong while the arm is still or slow, relaxed when it
//      moves fast -- edgeneuro::Vec3OneEuro), the elbow bend then taken between the two FILTERED vectors;
//   2. a small hysteresis on the base target, so a still arm leaves the base exactly still.
// Measured the same day: holding the arm forward the base swung over 383 us (std 102 us) while shoulder and elbow stayed
// within ~3 us -- the arm's own sway, about 30 us of base per degree of arm azimuth, not sensor noise (the MPU6050's
// 5 Hz DLPF removes that). The UART output (the humanoid path's input) stays the RAW readings; only the servos see this.
//
// STARTING VALUES, to be tuned on the arm with tools/measure_servo_response.py (std of the base while held forward,
// and how promptly a demo swing arrives):
constexpr float kFilterMinCutoffHz = 0.5f;   // still arm: ~0.3 s time constant
constexpr float kFilterBeta = 1.5f;          // per g/s of vector speed (~rad/s): a 2 rad/s swing -> ~3.5 Hz
constexpr unsigned kBaseHysteresisUs = 10u;  // ~1 degree of the base servo

// Angle (rad) between two gravity vectors -- the firmware's elbow bend. A missing reading (|v| <= 0.1 g) gives 0.
inline float elbow_bend_between(const pathb::Vec3& a, const pathb::Vec3& b) {
    const float ma = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float mb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (!(ma > 0.1f && mb > 0.1f)) return 0.0f;
    float c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (ma * mb);
    c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    return std::acos(c);
}

struct FilteredArm {
    pathb::Vec3 upper;   // all zero until the first real reading (drive::command answers that with rest)
    float elbow_bend;
};

class ArmInputFilter {
public:
    FilteredArm update(const pathb::Vec3& upper_raw, const pathb::Vec3& forearm_raw, float dt_s) {
        const auto u = upper_.update(upper_raw[0], upper_raw[1], upper_raw[2], dt_s);
        const auto f = forearm_.update(forearm_raw[0], forearm_raw[1], forearm_raw[2], dt_s);
        const pathb::Vec3 up{u.x, u.y, u.z}, fore{f.x, f.y, f.z};
        return {up, elbow_bend_between(up, fore)};
    }

private:
    Vec3OneEuro upper_{kFilterMinCutoffHz, kFilterBeta};
    Vec3OneEuro forearm_{kFilterMinCutoffHz, kFilterBeta};
};

// Holds its output until the input moves more than `band` away from it; the first input passes as is.
class Hysteresis {
public:
    unsigned apply(unsigned target, unsigned band) {
        const unsigned diff = target > held_ ? target - held_ : held_ - target;
        if (!initialized_ || diff > band) {
            held_ = target;
            initialized_ = true;
        }
        return held_;
    }

private:
    unsigned held_ = 0u;
    bool initialized_ = false;
};

// The base follows SLOWLY while the arm is being raised or lowered (2026-10-04, the user's choice; SESSION_LOG). Recorded
// on the real arm (data/arm_motion_20261004-021240.json): held still the base is steady, but on the way up or down --
// above all just after leaving the hanging pose, where the arm's azimuth is very sensitive to a small sideways offset --
// the base target jumped by up to ~1000 us. An accelerometer cannot tell an intended turn from the twist that comes with
// raising the arm, so this trades one error for another: during a raise the base lags; once the arm stops (or moves
// mainly sideways) it follows at once, so it always ends where it would have.
// Slow while, over the last kRaiseRateWindowS, the arm moved more than sensor noise AND either
//   * more up/down (change of tilt from HANG) than sideways -- decided by the DIRECTION of the motion, not its speed, so
//     a slow raise still counts (the user's point), or
//   * it is still near hanging (tilt < kNearHangDeg): there a raise's first moments look SIDEWAYS (the arm crosses the
//     HANG direction) and the azimuth is unreliable -- in the recording every wrong jump was below ~40 deg (raise
//     forward at 19.8 deg -> 1243 us, the diagonal at 24.6 deg -> 1599 us), and without this rule the base was pulled
//     to the wrong value first (787 us at 19.5 deg) and then held there by the slow follow.
// From the recording: raise speed (90th pct) 53-69 deg/s while raising, 10 deg/s in a sideways swing, 1.7 deg/s still.
constexpr float kRaiseRateWindowS = 0.2f;
constexpr float kRaiseNoiseDegPerS = 3.0f;   // above the still arm's 1.7 deg/s; a raise to horizontal in ~30 s still counts
constexpr float kRaiseFollowUsPerS = 150.0f; // simulated on the recording: raise forward 587 -> ~160 us of base swing
constexpr float kNearHangDeg = 40.0f;

class BaseRaiseFollow {
public:
    // target: the base pulse the drive asks for; upper: the (filtered) upper-arm vector it came from.
    unsigned apply(unsigned target, const pathb::Calibration* cal, const pathb::Vec3& upper, float dt_s) {
        float tilt = 0.0f, az = 0.0f;
        if (cal == nullptr || !cal->decode(upper, tilt, az)) {   // no usable reading: nothing to judge, pass it on
            count_ = 0;   // the readings before it say nothing about the motion after it
            out_ = static_cast<float>(target);
            return target;
        }
        pathb::Vec3 u{};
        pathb::detail::unit(upper, u);
        bool raising = false;
        if (count_ > 0) {
            // the reading about kRaiseRateWindowS ago (k back from the newest stored one; an entry's dt is the time
            // since the one before it)
            float span_s = dt_s;
            unsigned k = 0;
            while (span_s < kRaiseRateWindowS && k + 1u < count_) {
                span_s += hist_[(head_ + kHistory - 1u - k) % kHistory].dt;
                ++k;
            }
            const Entry& old = hist_[(head_ + kHistory - 1u - k) % kHistory];
            float c = u[0] * old.u[0] + u[1] * old.u[1] + u[2] * old.u[2];
            c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
            const float total = std::acos(c);
            const float raise = std::fabs(tilt - old.tilt);
            const float swing = std::sqrt(total * total > raise * raise ? total * total - raise * raise : 0.0f);
            const float noise = kRaiseNoiseDegPerS * (3.14159265f / 180.0f) * span_s;
            const bool near_hang = tilt < kNearHangDeg * (3.14159265f / 180.0f);
            raising = total > noise && (raise > swing || near_hang);
        }
        hist_[head_] = Entry{u, tilt, dt_s};
        head_ = (head_ + 1u) % kHistory;
        if (count_ < kHistory) ++count_;

        if (!raising) {                     // (always so for the first reading: no history yet)
            out_ = static_cast<float>(target);
        } else {
            const float step = kRaiseFollowUsPerS * dt_s;
            const float d = static_cast<float>(target) - out_;
            out_ += d > step ? step : (d < -step ? -step : d);
        }
        return static_cast<unsigned>(std::lround(out_));
    }

private:
    struct Entry {
        pathb::Vec3 u;
        float tilt;
        float dt;
    };
    static constexpr unsigned kHistory = 64u;   // >= kRaiseRateWindowS at the servo block's 10 ms cadence, with room
    Entry hist_[kHistory]{};
    unsigned head_ = 0u;
    unsigned count_ = 0u;
    float out_ = 0.0f;
};

}  // namespace edgeneuro::mearm

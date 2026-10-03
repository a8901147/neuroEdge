#pragma once

#include <cmath>

namespace edgeneuro {

// The 1-euro filter (Casiez, Roussel, Vogel, "1 Euro Filter: A Simple Speed-based Low-pass Filter for Noisy Input in
// Interactive Systems", CHI 2012) on a 3-axis vector -- an IMU's raw accel, for the real MEArm's servo path
// (2026-10-02: the base swung back and forth on tiny arm motions; a fixed low-pass must trade jitter at rest against
// lag in motion, this one raises its cutoff with speed instead).
//
// Each update, with a first-order low-pass  lp(prev, in, alpha) = prev + alpha * (in - prev),
// alpha(fc) = 1 / (1 + tau / dt), tau = 1 / (2 pi fc):
//   speed  = | lp_d( (in - out_prev) / dt ) |        (vector velocity, itself low-passed at d_cutoff_hz)
//   cutoff = min_cutoff_hz + beta * speed
//   out    = lp(out_prev, in, alpha(cutoff))
// ONE speed for the whole vector, so every axis gets the same cutoff (per-axis cutoffs would distort the direction,
// i.e. the base's azimuth). For a unit gravity vector the speed is about the arm's rotation rate in rad/s.
// The first real reading seeds the output as-is; the firmware's all-zero "no reading yet" vector passes through without
// seeding; a non-finite reading or a non-positive/non-finite dt is ignored. No allocation.
class Vec3OneEuro {
public:
    struct Vec {
        float x, y, z;
    };

    Vec3OneEuro(float min_cutoff_hz, float beta, float d_cutoff_hz = 1.0f) noexcept
        : min_cutoff_(min_cutoff_hz), beta_(beta), d_cutoff_(d_cutoff_hz) {}

    Vec update(float x, float y, float z, float dt_s) noexcept {
        if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
            return out_;
        }
        if (!initialized_) {
            if (x == 0.0f && y == 0.0f && z == 0.0f) {
                return {0.0f, 0.0f, 0.0f};
            }
            out_ = {x, y, z};
            initialized_ = true;
            return out_;
        }
        if (!(std::isfinite(dt_s) && dt_s > 0.0f)) {
            return out_;
        }
        const float ad = alpha(d_cutoff_, dt_s);
        vel_.x += ad * ((x - out_.x) / dt_s - vel_.x);
        vel_.y += ad * ((y - out_.y) / dt_s - vel_.y);
        vel_.z += ad * ((z - out_.z) / dt_s - vel_.z);
        const float speed = std::sqrt(vel_.x * vel_.x + vel_.y * vel_.y + vel_.z * vel_.z);
        const float a = alpha(min_cutoff_ + beta_ * speed, dt_s);
        out_.x += a * (x - out_.x);
        out_.y += a * (y - out_.y);
        out_.z += a * (z - out_.z);
        return out_;
    }

    bool initialized() const noexcept { return initialized_; }

private:
    static float alpha(float cutoff_hz, float dt_s) noexcept {
        if (!(cutoff_hz > 0.0f)) return 1.0f;                  // no (or a nonsensical) cutoff: no smoothing
        const float tau = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
        return 1.0f / (1.0f + tau / dt_s);
    }

    float min_cutoff_, beta_, d_cutoff_;
    Vec out_{0.0f, 0.0f, 0.0f};
    Vec vel_{0.0f, 0.0f, 0.0f};
    bool initialized_ = false;
};

}  // namespace edgeneuro

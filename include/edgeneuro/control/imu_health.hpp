#pragma once

#include <array>
#include <cmath>

namespace edgeneuro {

// Firmware-side IMU health check (2026-09-28), so the REAL arm never follows a failed sensor. Built from the two real
// failures of 2026-09-27 (SESSION_LOG): the upper-arm MPU6050 dropping out -- its variable then keeps the last value, or a
// garbage one (0.21 g, 2.2 g) -- and the forearm one stuck at (1.999939, 0, 0) while its I2C reads still "completed", so
// the firmware's own completion counters could not see it. Same rules as tools/sensor_health.py:
//   * implausible reading (non-finite, an axis at full scale +-1.99 g, |a| outside 0.3..3 g): a fault AT ONCE;
//   * a value that does not change for kFrozenUpdates updates: a fault (a live accelerometer is never bit-identical);
//   * healthy again after kHealthyStreak consecutive plausible, changing readings (also how a new monitor starts).
// Call update() once per servo cycle (100 Hz in phase3_control_loop). A sensor declared optional is never checked.
class ImuHealth {
public:
    using Vec3 = std::array<float, 3>;
    static constexpr int kFrozenUpdates = 30;        // 0.3 s at 100 Hz
    static constexpr int kHealthyStreak = 3;

    constexpr explicit ImuHealth(bool check_upper_arm = true, bool check_forearm = true) noexcept
        : upper_{check_upper_arm}, fore_{check_forearm} {}

    void update(const Vec3& upper_arm, const Vec3& forearm) noexcept {
        upper_.update(upper_arm);
        fore_.update(forearm);
    }

    bool upper_arm_fault() const noexcept { return upper_.fault(); }
    bool forearm_fault() const noexcept { return fore_.fault(); }
    bool ok() const noexcept { return !upper_.fault() && !fore_.fault(); }

private:
    struct Channel {
        bool checked;
        Vec3 last{};
        bool has_last = false;
        int same_run = 0;
        int good_streak = 0;

        static bool plausible(const Vec3& v) noexcept {
            float mag2 = 0.0f;
            for (float c : v) {
                if (!std::isfinite(c) || std::fabs(c) >= 1.99f) return false;
                mag2 += c * c;
            }
            return mag2 >= 0.3f * 0.3f && mag2 <= 3.0f * 3.0f;
        }

        void update(const Vec3& v) noexcept {
            if (!checked) return;
            same_run = (has_last && v == last) ? same_run + 1 : 1;
            last = v;
            has_last = true;
            if (!plausible(v) || same_run >= kFrozenUpdates) {
                good_streak = 0;
            } else if (good_streak < kHealthyStreak) {
                ++good_streak;
            }
        }

        bool fault() const noexcept { return checked && good_streak < kHealthyStreak; }
    };

    Channel upper_;
    Channel fore_;
};

}  // namespace edgeneuro

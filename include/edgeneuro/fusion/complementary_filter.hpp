#pragma once

#include <cmath>

namespace edgeneuro {

// Complementary filter: fuses a gyroscope (accurate short-term, drifts
// without bound over time since it only measures rate) with an
// accelerometer (noisy sample-to-sample, but accurate on average since it
// just measures the gravity vector) into a drift-corrected roll/pitch
// estimate.
//
// This intentionally does NOT satisfy the `Filter` concept in concepts.hpp
// (that concept is a per-channel scalar process(value)->value transform).
// Orientation fusion is inherently cross-channel -- it needs accel x/y/z
// and gyro rate together, plus a time step -- so it lives as a standalone
// component rather than one of Pipeline's per-channel EmgFilterT/ImuFilterT
// slots. Wiring it into the hot loop (as a post-processing stage on the raw
// IMU channels) is future work, not required for this to be useful and
// testable on its own.
//
// No magnetometer input, so yaw is not observable from this filter alone --
// only roll and pitch are estimated. This matches the MPU6050 (accelerometer
// + gyroscope only, no compass) this project targets: yaw would need either
// a magnetometer-equipped IMU (e.g. MPU9250) or acceptance of unbounded yaw
// drift, not something this filter can fix.
//
//   angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
//
// alpha close to 1 trusts the gyro-integrated estimate more (smoother, but
// slower to correct drift); alpha close to 0 trusts the fresh accelerometer
// reading more (corrects drift fast, but noisier).
//
// Caller contract: gyro_x/gyro_y must be in rad/s and dt in seconds (both
// linear in the update equation -- getting the units wrong just rescales
// the integrated angle, it won't produce garbage, but it will be wrong).
// accel_x/y/z can be in any consistent unit: only ever used via atan2, so
// absolute scale cancels out.
template <typename ValueType>
class ComplementaryFilter {
public:
    ComplementaryFilter() noexcept
        : ComplementaryFilter(static_cast<ValueType>(0.98), static_cast<ValueType>(0.01)) {}

    ComplementaryFilter(ValueType alpha, ValueType dt) noexcept : alpha_(alpha), dt_(dt) {}

    // gyro_x/gyro_y: angular rate about the X/Y axes (rad/s).
    // accel_x/y/z: accelerometer reading (the gravity vector, when the
    // sensor isn't also undergoing significant linear acceleration).
    void update(ValueType gyro_x, ValueType gyro_y, ValueType accel_x, ValueType accel_y, ValueType accel_z) noexcept {
        const ValueType accel_roll = accel_roll_angle(accel_y, accel_z);
        const ValueType accel_pitch = accel_pitch_angle(accel_x, accel_y, accel_z);

        roll_ = alpha_ * (roll_ + gyro_x * dt_) + (ValueType{1} - alpha_) * accel_roll;
        pitch_ = alpha_ * (pitch_ + gyro_y * dt_) + (ValueType{1} - alpha_) * accel_pitch;
    }

    // Seeds roll/pitch directly from the accelerometer-derived angle,
    // skipping the multi-update convergence transient that starting from
    // roll=pitch=0 would otherwise cause. At alpha=0.98 that transient's
    // time constant is ~50 update()s -- found while validating this filter
    // against real EMG-EPN-612 IMU data, where a cold start visibly took a
    // real chunk of a 5-second recording to settle toward the
    // accelerometer-implied angle. Call once before the first update(),
    // not as a replacement for it.
    void initialize(ValueType accel_x, ValueType accel_y, ValueType accel_z) noexcept {
        roll_ = accel_roll_angle(accel_y, accel_z);
        pitch_ = accel_pitch_angle(accel_x, accel_y, accel_z);
    }

    ValueType roll() const noexcept { return roll_; }
    ValueType pitch() const noexcept { return pitch_; }

    void reset() noexcept {
        roll_ = ValueType{0};
        pitch_ = ValueType{0};
    }

private:
    static ValueType accel_roll_angle(ValueType accel_y, ValueType accel_z) noexcept {
        return std::atan2(accel_y, accel_z);
    }

    static ValueType accel_pitch_angle(ValueType accel_x, ValueType accel_y, ValueType accel_z) noexcept {
        return std::atan2(-accel_x, std::sqrt(accel_y * accel_y + accel_z * accel_z));
    }

    ValueType alpha_;
    ValueType dt_;
    ValueType roll_{0};
    ValueType pitch_{0};
};

} // namespace edgeneuro

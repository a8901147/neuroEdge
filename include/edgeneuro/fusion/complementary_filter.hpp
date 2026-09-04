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
// slots.
//
// It also never will go through Pipeline's ImuFilterT slot, by design, not
// just for lack of a concept match: Pipeline's window/feature/classify path
// only produces a result once every WindowSize samples (e.g. 200ms at
// WindowSize=200 @ 1kHz), which is fine for EMG's "did the user hold a
// contraction long enough" decision but far too laggy for orientation
// control that's supposed to track the user's real arm continuously (see
// PRD.md Section 3's Phase 3 control-architecture note). The intended
// caller reads a sensor, calls update() once per fresh reading, and feeds
// roll()/pitch() straight to an actuator setpoint -- bypassing Pipeline
// entirely, not feeding into it.
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
    // Uses the constructor's fixed dt_ -- only correct when update() is
    // actually called at that exact fixed interval. Stage 5b's combined
    // EMG+IMU loop calls update() whenever a non-blocking I2C read
    // happens to complete, which isn't fixed-interval (varies ~2-4 EMG
    // ticks depending on bus timing) -- that caller must use the
    // dt-overload below instead, or every gyro integration step would be
    // silently wrong by whatever factor the real interval differs from
    // dt_.
    void update(ValueType gyro_x, ValueType gyro_y, ValueType accel_x, ValueType accel_y, ValueType accel_z) noexcept {
        update(gyro_x, gyro_y, accel_x, accel_y, accel_z, dt_);
    }

    // Same as above but with an explicit per-call dt (seconds), overriding
    // the constructor's fixed dt_ for this one call -- for callers whose
    // update() cadence isn't actually fixed-interval.
    void update(ValueType gyro_x, ValueType gyro_y, ValueType accel_x, ValueType accel_y, ValueType accel_z,
                ValueType dt) noexcept {
        const ValueType accel_roll = accel_roll_angle(accel_y, accel_z);
        const ValueType accel_pitch = accel_pitch_angle(accel_x, accel_y, accel_z);

        roll_ = alpha_ * (roll_ + gyro_x * dt) + (ValueType{1} - alpha_) * accel_roll;
        pitch_ = alpha_ * (pitch_ + gyro_y * dt) + (ValueType{1} - alpha_) * accel_pitch;
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

    // 2026-09-04 FIXED: was atan2(-accel_x, sqrt(accel_y^2+accel_z^2)).
    // sqrt() is always >= 0, which mathematically restricts atan2's output
    // to [-90deg,+90deg] no matter the true rotation -- past that, a real,
    // physically continuous rotation reflects back into that range instead
    // of continuing, so two genuinely different angles (e.g. a forward-
    // raise and a backward-extension both past ~90deg from rest) can
    // decode to the IDENTICAL pitch. Found 2026-09-04 from a real hardware
    // session where exactly that happened; proven with a pure synthetic
    // rotation sweep in tests/test_complementary_filter.cpp (not tied to
    // that session's specific sensor mount).
    //
    // Fixed by dropping accel_y from the denominator entirely -- same
    // simplification accel_roll_angle below already makes (it uses only
    // accel_y/accel_z, ignoring accel_x), which is why roll never had this
    // problem: a plain two-argument atan2(-accel_x, accel_z) carries full
    // sign information in both arguments, so it has the same unrestricted
    // +-180deg range as accel_roll_angle, with no reflection point. Trade-
    // off: pitch is now less accurate during LARGE simultaneous roll
    // (accel_y no longer contributes to leveling the reference), the same
    // known limitation accel_roll_angle already accepted for accel_x --
    // a deliberate, precedented choice, not an oversight, and a far
    // smaller cost than a hard sign ambiguity on the primary flexion/
    // extension axis.
    static ValueType accel_pitch_angle(ValueType accel_x, ValueType /*accel_y*/, ValueType accel_z) noexcept {
        return std::atan2(-accel_x, accel_z);
    }

    ValueType alpha_;
    ValueType dt_;
    ValueType roll_{0};
    ValueType pitch_{0};
};

} // namespace edgeneuro

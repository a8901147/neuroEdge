// Tested purely against known physics (unit vectors, constant analytic
// rotation rates), the same way test_filters.cpp validates IirFilter --
// not against the real EMG-EPN-612 dataset's IMU columns, since this
// project doesn't have verified axis conventions or gyro units (deg/s vs
// rad/s) for that Myo armband hardware. Making up an answer to check
// against would be exactly the kind of unverified guess this project
// avoids (see PRD's Phase 1.5 notes on the EMG-EPN-612 schema-guessing
// episode) -- so this stays a self-contained numerical test of the
// algorithm, independent of any specific sensor's data format.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <numbers>

#include "edgeneuro/fusion/complementary_filter.hpp"

using Catch::Approx;
using edgeneuro::ComplementaryFilter;

TEST_CASE("ComplementaryFilter at rest (gravity on Z, no rotation) stays at zero roll/pitch", "[fusion]") {
    ComplementaryFilter<float> filter;
    for (int i = 0; i < 50; ++i) {
        filter.update(0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    }
    REQUIRE(filter.roll() == Approx(0.0f).margin(1e-6));
    REQUIRE(filter.pitch() == Approx(0.0f).margin(1e-6));
}

TEST_CASE("ComplementaryFilter with alpha=0 reproduces the accelerometer-only roll angle", "[fusion]") {
    // alpha=0 means the gyro-integrated term has zero weight, so one
    // update() must land exactly on atan2(accel_y, accel_z) regardless of
    // gyro input.
    ComplementaryFilter<float> filter(0.0f, 0.01f);
    filter.update(/*gyro_x=*/5.0f, /*gyro_y=*/0.0f, /*accel=*/0.0f, 1.0f, 0.0f);
    REQUIRE(filter.roll() == Approx(std::numbers::pi_v<float> / 2.0f));
}

TEST_CASE("ComplementaryFilter with alpha=0 reproduces the accelerometer-only pitch angle", "[fusion]") {
    ComplementaryFilter<float> filter(0.0f, 0.01f);
    filter.update(/*gyro_x=*/0.0f, /*gyro_y=*/5.0f, /*accel=*/1.0f, 0.0f, 0.0f);
    REQUIRE(filter.pitch() == Approx(-std::numbers::pi_v<float> / 2.0f));
}

TEST_CASE("ComplementaryFilter with alpha=1 is pure gyro integration", "[fusion]") {
    // alpha=1 zeroes out the accelerometer term entirely, so this must
    // match plain rectangular integration: angle = rate * dt * steps.
    constexpr float kRate = 1.0f;   // rad/s
    constexpr float kDt = 0.1f;     // s
    constexpr int kSteps = 10;
    ComplementaryFilter<float> filter(1.0f, kDt);
    for (int i = 0; i < kSteps; ++i) {
        filter.update(kRate, 0.0f, 0.0f, 0.0f, 1.0f); // accel ignored at alpha=1
    }
    REQUIRE(filter.roll() == Approx(kRate * kDt * kSteps).epsilon(0.001));
}

TEST_CASE("ComplementaryFilter bounds a constant gyro bias that pure integration lets grow without bound", "[fusion]") {
    // A real gyro has a small constant bias even when stationary. Pure
    // integration (alpha=1) accumulates it linearly forever; the whole
    // point of the complementary filter is that a nonzero (1-alpha)
    // accelerometer term bounds this, converging to a fixed steady-state
    // offset instead of drifting forever. Accelerometer input stays at
    // true level (0,0,1) throughout, so its "correction" always pulls
    // toward roll=0.
    constexpr float kBias = 0.05f;  // rad/s, simulated constant gyro bias
    constexpr float kDt = 0.01f;
    constexpr int kSteps = 4000;

    ComplementaryFilter<float> integrate_only(1.0f, kDt);
    ComplementaryFilter<float> corrected(0.98f, kDt);
    for (int i = 0; i < kSteps; ++i) {
        integrate_only.update(kBias, 0.0f, 0.0f, 0.0f, 1.0f);
        corrected.update(kBias, 0.0f, 0.0f, 0.0f, 1.0f);
    }

    // Pure integration: unbounded, exactly bias*dt*steps.
    REQUIRE(integrate_only.roll() == Approx(kBias * kDt * kSteps).epsilon(0.001));

    // Complementary filter: converges to the fixed point of
    // roll = alpha*(roll + bias*dt) + (1-alpha)*0, i.e.
    // roll* = alpha*bias*dt / (1-alpha) -- bounded and independent of
    // kSteps once settled, unlike the integrate-only case above.
    constexpr float kAlpha = 0.98f;
    const float expected_steady_state = kAlpha * kBias * kDt / (1.0f - kAlpha);
    REQUIRE(corrected.roll() == Approx(expected_steady_state).epsilon(0.01));

    // The whole point: the corrected estimate is far smaller than the
    // unbounded integration would produce after the same number of steps.
    REQUIRE(corrected.roll() < integrate_only.roll() / 10.0f);
}

TEST_CASE("ComplementaryFilter::initialize seeds state without needing update() to converge", "[fusion]") {
    // Same accel vector as the alpha=0 roll test above, but here alpha=0.98
    // (the "trust the gyro" default) -- without initialize(), a single
    // update() would barely move roll_ away from 0. initialize() must land
    // exactly on the accelerometer-derived angle immediately.
    ComplementaryFilter<float> filter(0.98f, 0.01f);
    filter.initialize(/*accel=*/0.0f, 1.0f, 0.0f);
    REQUIRE(filter.roll() == Approx(std::numbers::pi_v<float> / 2.0f));
    REQUIRE(filter.pitch() == Approx(0.0f).margin(1e-6));
}

TEST_CASE("ComplementaryFilter::reset clears roll/pitch state", "[fusion]") {
    ComplementaryFilter<float> filter(1.0f, 0.1f);
    filter.update(1.0f, 1.0f, 0.0f, 0.0f, 1.0f);
    REQUIRE(filter.roll() != Approx(0.0f));
    filter.reset();
    REQUIRE(filter.roll() == Approx(0.0f).margin(1e-6));
    REQUIRE(filter.pitch() == Approx(0.0f).margin(1e-6));
}

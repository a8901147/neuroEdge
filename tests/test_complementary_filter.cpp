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
#include <cmath>
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

TEST_CASE("ComplementaryFilter's per-call dt overload uses the passed dt, not the constructor's", "[fusion]") {
    // Stage 5b's combined EMG+IMU loop calls update() at an irregular
    // cadence (whenever a non-blocking I2C read happens to finish), so it
    // must be able to override dt per call -- constructed with a dt that,
    // if silently used instead, would give a visibly different answer.
    constexpr float kRate = 1.0f;
    constexpr float kWrongDt = 0.1f;
    constexpr float kRealDt = 0.5f;
    ComplementaryFilter<float> filter(1.0f, kWrongDt); // alpha=1: pure gyro integration
    filter.update(kRate, 0.0f, 0.0f, 0.0f, 1.0f, kRealDt);
    REQUIRE(filter.roll() == Approx(kRate * kRealDt).epsilon(0.001));
    REQUIRE(filter.roll() != Approx(kRate * kWrongDt).epsilon(0.001));
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

// 2026-09-04: added after a real hardware session found firmware's shoulder
// pitch/roll flipping sign between a forward-raise and a backward-extension
// that should have been opposite -- traced to this exact formula, not the
// sensor or the specific mount (see PRD.md/git history for the full
// derivation). Kept in this file, tested the same sensor-format-independent
// way as the rest of it (a pure synthetic rotation, not real captured
// data): initialize() from a synthetic gravity vector rotated by a known
// angle about the Y axis (accel = (sin(theta), 0, cos(theta)), so
// theta=0 matches this file's own "gravity on Z" rest convention) and read
// back pitch().
TEST_CASE("ComplementaryFilter pitch stays monotonic across a full real shoulder ROM sweep", "[fusion][bug]") {
    auto pitch_for_angle_deg = [](float deg) {
        ComplementaryFilter<float> filter;
        const float rad = deg * std::numbers::pi_v<float> / 180.0f;
        filter.initialize(std::sin(rad), 0.0f, std::cos(rad));
        return filter.pitch();
    };

    // Sharpest, most concrete demonstration: 70deg and 110deg are two
    // clearly DIFFERENT real rotations (40deg apart), well within a real
    // shoulder's flexion range (up to ~150-180deg) -- but
    // accel_pitch_angle's atan2(-ax, sqrt(ay^2+az^2)) has a second
    // argument that's a sqrt (always >= 0), which mathematically restricts
    // its output to [-90deg,+90deg]. Past 90deg the true angle reflects
    // instead of continuing, so these two distinct real angles currently
    // decode to the IDENTICAL pitch -- this REQUIRE describes the CORRECT
    // behavior (they must differ) and is expected to FAIL against today's
    // formula; it should start passing once accel_pitch_angle is replaced
    // with a wide-range formulation (see PRD.md's swing/twist notes).
    REQUIRE(pitch_for_angle_deg(110.0f) != Approx(pitch_for_angle_deg(70.0f)).margin(1e-3));

    // Full-range monotonicity: a real shoulder's flexion/extension sweep
    // (roughly -60deg extension to +160deg flexion, see
    // tools/mujoco_bridge/run_demo_live.py's SHOULDER_PITCH_RANGE comment
    // for the anatomical reference) should decode to a pitch that moves
    // the same direction throughout -- no reflecting back partway through
    // a real, physically continuous motion. Also expected to fail today.
    constexpr int kStartDeg = -60;
    constexpr int kEndDeg = 160;
    constexpr int kStepDeg = 5;
    float prev = pitch_for_angle_deg(static_cast<float>(kStartDeg));
    for (int deg = kStartDeg + kStepDeg; deg <= kEndDeg; deg += kStepDeg) {
        const float current = pitch_for_angle_deg(static_cast<float>(deg));
        REQUIRE(current < prev); // this formula's sign convention: pitch decreases as angle increases
        prev = current;
    }
}

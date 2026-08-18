#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/slew_rate_limiter.hpp"

using Catch::Approx;
using edgeneuro::SlewRateLimiter;

TEST_CASE("SlewRateLimiter starts at the given initial value", "[control]") {
    SlewRateLimiter<float> limiter(1.0f, /*initial=*/0.25f);
    REQUIRE(limiter.value() == Approx(0.25f));
}

TEST_CASE("SlewRateLimiter reaching a target within one step lands exactly on it, no overshoot", "[control]") {
    // max_rate=1.0/s, dt=1.0s -> max_step=1.0, target is only 0.3 away.
    SlewRateLimiter<float> limiter(1.0f);
    const float out = limiter.update(0.3f, 1.0f);
    REQUIRE(out == Approx(0.3f));
    REQUIRE(limiter.value() == Approx(0.3f));
}

TEST_CASE("SlewRateLimiter caps the step when the target is far away", "[control]") {
    // max_rate=1.0/s, dt=0.1s -> max_step=0.1 per call. Target is 1.0 away.
    SlewRateLimiter<float> limiter(1.0f);
    const float out = limiter.update(1.0f, 0.1f);
    REQUIRE(out == Approx(0.1f));
    REQUIRE(limiter.value() == Approx(0.1f));
}

TEST_CASE("SlewRateLimiter reaches a step target over several updates without overshoot", "[control]") {
    // GripStateMachine-style 0->1 step, max_rate chosen for a 0.2s ramp.
    SlewRateLimiter<float> limiter(5.0f); // 1.0 / 0.2s = 5.0/s
    for (int i = 0; i < 19; ++i) {
        const float out = limiter.update(1.0f, 0.01f);
        REQUIRE(out <= 1.0f); // never overshoots past the target
    }
    // Comfortably past the 0.2s ramp -- must have arrived, not still climbing.
    REQUIRE(limiter.update(1.0f, 0.01f) == Approx(1.0f));
}

TEST_CASE("SlewRateLimiter ramps down symmetrically when the target decreases", "[control]") {
    SlewRateLimiter<float> limiter(1.0f, /*initial=*/1.0f);
    const float out = limiter.update(0.0f, 0.1f);
    REQUIRE(out == Approx(0.9f));
}

TEST_CASE("SlewRateLimiter::reset jumps to the given value immediately", "[control]") {
    SlewRateLimiter<float> limiter(1.0f);
    limiter.update(0.05f, 0.1f); // partway toward 1.0, but not there
    limiter.reset(0.5f);
    REQUIRE(limiter.value() == Approx(0.5f));
}

TEST_CASE("SlewRateLimiter tracks a target that keeps moving, staying within max_rate of it", "[control]") {
    // Simulates a continuously-changing setpoint (e.g. IMU-driven position)
    // rather than a single step -- output should trail the target but
    // never jump by more than max_rate * dt in a single update.
    SlewRateLimiter<float> limiter(2.0f); // 2.0/s
    float target = 0.0f;
    float prev = limiter.value();
    for (int i = 0; i < 50; ++i) {
        target += 0.05f; // ramps target up faster than the limiter can fully track early on
        const float out = limiter.update(target, 0.01f);
        REQUIRE(out - prev <= Approx(2.0f * 0.01f).margin(1e-6f));
        prev = out;
    }
}

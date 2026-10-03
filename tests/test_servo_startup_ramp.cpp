// ServoStartupRamp: what keeps a real servo from being commanded straight from its
// rest pulse to wherever the sensors say, on the first tick after power-up.
//
// Open-loop hobby servos have no position feedback and do not return anywhere when
// unpowered, so the very FIRST pulse after power-up moves the servo from wherever it
// was at the servo's own full speed -- nothing here can slow that. What this does
// control is everything after it: the output starts at the rest pulse and walks
// toward the target at a slow "start" rate until it has arrived, and only then is
// allowed to follow at the fast "track" rate.
//
// Rest (1500us) and the two rates are PLACEHOLDERS (SESSION_LOG 2026-09-26), not
// measured on the real arm. Tests assert properties, not those numbers.
#include <cmath>
#include <limits>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/servo_startup_ramp.hpp"

using edgeneuro::ServoStartupRamp;

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kDt = 0.001f;   // the firmware's 1 kHz tick

// rest 1500, range 500..2500, start 300 us/s, track 6000 us/s, arrived within 2 us
ServoStartupRamp make() { return ServoStartupRamp(1500u, 500u, 2500u, 300.0f, 6000.0f, 2.0f); }
}  // namespace

TEST_CASE("the ramp starts at the rest pulse", "[servo][ramp]") {
    auto r = make();
    REQUIRE(r.current_us() == 1500u);
    REQUIRE_FALSE(r.tracking());
}

TEST_CASE("before arriving, the output never moves faster than the start rate", "[servo][ramp]") {
    auto r = make();
    float prev = 1500.0f;
    for (int i = 0; i < 3000; ++i) {
        const unsigned out = r.step(2400.0f, kDt);
        REQUIRE(std::fabs(static_cast<float>(out) - prev) <= 300.0f * kDt + 1.0f);   // +1: integer rounding
        prev = static_cast<float>(out);
        if (r.tracking()) break;
    }
}

TEST_CASE("a start rate below one microsecond per tick still makes progress", "[servo][ramp]") {
    // 300 us/s * 1 ms = 0.3 us per tick: an implementation that rounds its STATE to
    // whole microseconds every tick would never move at all.
    auto r = make();
    for (int i = 0; i < 1000; ++i) r.step(2000.0f, kDt);     // one second
    REQUIRE(r.current_us() >= 1790u);                        // ~ 1500 + 300
    REQUIRE(r.current_us() <= 1810u);
}

TEST_CASE("the output walks to the target and never overshoots it", "[servo][ramp]") {
    auto r = make();
    unsigned last = 1500u;
    for (int i = 0; i < 20000; ++i) {
        last = r.step(1800.0f, kDt);
        REQUIRE(last <= 1800u);
        REQUIRE(last >= 1500u);
    }
    REQUIRE(last == 1800u);
    REQUIRE(r.tracking());
}

TEST_CASE("once arrived it follows a new target at the fast track rate, not the slow start rate", "[servo][ramp]") {
    auto r = make();
    for (int i = 0; i < 20000; ++i) r.step(1600.0f, kDt);
    REQUIRE(r.tracking());
    const unsigned before = r.current_us();
    for (int i = 0; i < 50; ++i) r.step(2400.0f, kDt);       // 50 ms
    // at 6000 us/s that is up to 300 us; at the 300 us/s start rate it would be 15
    REQUIRE(r.current_us() - before > 100u);
}

TEST_CASE("tracking is latched: moving away from the target does not fall back to the slow rate", "[servo][ramp]") {
    auto r = make();
    for (int i = 0; i < 20000; ++i) r.step(1600.0f, kDt);
    REQUIRE(r.tracking());
    r.step(2400.0f, kDt);
    r.step(500.0f, kDt);
    REQUIRE(r.tracking());
}

TEST_CASE("the output stays inside the servo's range for any target", "[servo][ramp]") {
    for (float target : {-1e9f, -1.0f, 0.0f, 499.0f, 2501.0f, 1e9f, kInf, -kInf}) {
        auto r = make();
        for (int i = 0; i < 20000; ++i) {
            const unsigned out = r.step(target, kDt);
            REQUIRE(out >= 500u);
            REQUIRE(out <= 2500u);
        }
    }
}

TEST_CASE("a NaN target holds the current pulse", "[servo][ramp]") {
    auto r = make();
    for (int i = 0; i < 500; ++i) r.step(2000.0f, kDt);
    const unsigned held = r.current_us();
    for (int i = 0; i < 100; ++i) REQUIRE(r.step(kNaN, kDt) == held);
    r.step(2000.0f, kDt);                                    // and it carries on afterwards
    REQUIRE(r.current_us() >= held);
}

TEST_CASE("a bad time step holds the current pulse", "[servo][ramp]") {
    auto r = make();
    for (int i = 0; i < 500; ++i) r.step(2000.0f, kDt);
    const unsigned held = r.current_us();
    for (float dt : {0.0f, -0.001f, kNaN, kInf, -kInf}) REQUIRE(r.step(2000.0f, dt) == held);
}

TEST_CASE("a rest pulse outside the servo's range is clamped into it", "[servo][ramp]") {
    ServoStartupRamp low(100u, 500u, 2500u, 300.0f, 6000.0f, 2.0f);
    ServoStartupRamp high(9000u, 500u, 2500u, 300.0f, 6000.0f, 2.0f);
    REQUIRE(low.current_us() == 500u);
    REQUIRE(high.current_us() == 2500u);
}

TEST_CASE("a target already at the rest pulse is 'arrived' immediately", "[servo][ramp]") {
    auto r = make();
    r.step(1500.0f, kDt);
    REQUIRE(r.tracking());
    REQUIRE(r.current_us() == 1500u);
}

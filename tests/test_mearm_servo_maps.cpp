// Safety properties of the four real-arm servo maps (firmware/src/
// phase3_control_loop_main.cpp drives TIM3 CCR1..4 straight from these).
// Bounds are the ranges measured on the assembled MEArm with
// servo_limit_finder_4ch (SESSION_LOG 2026-09-21/22) -- written here as
// independent literals on purpose: if someone edits the maps, these must
// not silently follow. Properties, not pinned interior values, so a
// legitimate retune of value ranges doesn't fight the tests.
#include <cmath>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_servo_maps.hpp"

namespace {

struct Real { const edgeneuro::ServoAngleMap& map; unsigned lo, hi; const char* name; };

const Real kArm[] = {
    {edgeneuro::mearm::base_map(),     500u, 2500u, "base"},
    {edgeneuro::mearm::shoulder_map(), 1200u, 2100u, "shoulder"},
    {edgeneuro::mearm::elbow_map(),     500u, 1850u, "elbow"},
    {edgeneuro::mearm::claw_map(),     1300u, 1500u, "claw"},   // limited to 1300..1500 (user, 2026-10-03)
};

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

}  // namespace

TEST_CASE("every real-arm map stays inside its measured range for any finite input", "[mearm]") {
    for (const auto& j : kArm) {
        INFO(j.name);
        for (float v = -100.0f; v <= 100.0f; v += 0.037f) {
            const unsigned p = j.map.pulse_us(v);
            REQUIRE(p >= j.lo);
            REQUIRE(p <= j.hi);
        }
    }
}

TEST_CASE("every real-arm map stays inside its measured range for +-infinity", "[mearm]") {
    for (const auto& j : kArm) {
        INFO(j.name);
        for (float v : {kInf, -kInf, std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::lowest()}) {
            const unsigned p = j.map.pulse_us(v);
            REQUIRE(p >= j.lo);
            REQUIRE(p <= j.hi);
        }
    }
}

TEST_CASE("a NaN sensor value (e.g. a filter fed a bad sample) never yields an out-of-range pulse", "[mearm]") {
    // static_cast<unsigned>(NaN) is undefined behaviour; on the real chip that
    // could become 0 or 0xFFFFFFFF -> a servo slammed into its stop.
    for (const auto& j : kArm) {
        INFO(j.name);
        const unsigned p = j.map.pulse_us(kNaN);
        REQUIRE(p >= j.lo);
        REQUIRE(p <= j.hi);
    }
}

TEST_CASE("every real-arm map is monotonic in its input", "[mearm]") {
    for (const auto& j : kArm) {
        INFO(j.name);
        long direction = 0;
        unsigned prev = j.map.pulse_us(-100.0f);
        for (float v = -100.0f; v <= 100.0f; v += 0.05f) {
            const unsigned p = j.map.pulse_us(v);
            const long step = static_cast<long>(p) - static_cast<long>(prev);
            if (step != 0) {
                if (direction == 0) direction = step > 0 ? 1 : -1;
                REQUIRE((step > 0 ? 1 : -1) == direction);
            }
            prev = p;
        }
    }
}

TEST_CASE("each map reaches both ends of its measured range", "[mearm]") {
    for (const auto& j : kArm) {
        INFO(j.name);
        const unsigned a = j.map.pulse_us(-100.0f), b = j.map.pulse_us(100.0f);
        REQUIRE(((a == j.lo && b == j.hi) || (a == j.hi && b == j.lo)));
    }
}

// ---- start-up ramps (one per servo): rest pulse + slow walk, inside the measured range ----

TEST_CASE("every real-arm ramp starts at its own rest pulse, inside its measured range", "[mearm][ramp]") {
    // rest pulses (2026-09-26, chosen by looking at the real arm): base/shoulder/elbow 1500,
    // claw 1300 = open. Independent literals on purpose: not read back from the header.
    const struct { edgeneuro::ServoStartupRamp ramp; unsigned rest, lo, hi; const char* name; } arm[] = {
        {edgeneuro::mearm::base_ramp(), 1500u, 500u, 2500u, "base"},
        {edgeneuro::mearm::shoulder_ramp(), 1500u, 1200u, 2100u, "shoulder"},
        {edgeneuro::mearm::elbow_ramp(), 1500u, 500u, 1850u, "elbow"},
        {edgeneuro::mearm::claw_ramp(), 1300u, 1300u, 1500u, "claw"},
    };
    for (const auto& j : arm) {
        INFO(j.name);
        REQUIRE(j.ramp.current_us() == j.rest);
        REQUIRE(j.ramp.current_us() >= j.lo);
        REQUIRE(j.ramp.current_us() <= j.hi);
    }
}

TEST_CASE("the claw rests where a relaxed hand (grip = 0) already sends it, so it does not move at boot", "[mearm][ramp]") {
    REQUIRE(edgeneuro::mearm::claw_ramp().current_us() == edgeneuro::mearm::claw_map().pulse_us(0.0f));
}

TEST_CASE("no real-arm ramp ever leaves its servo's measured range, whatever it is told", "[mearm][ramp]") {
    const struct { unsigned lo, hi; const char* name; } arm[] = {
        {500u, 2500u, "base"}, {1200u, 2100u, "shoulder"}, {500u, 1850u, "elbow"}, {1300u, 1500u, "claw"}};
    for (int j = 0; j < 4; ++j) {
        for (float target : {-1e9f, 0.0f, 1500.0f, 3000.0f, 1e9f, kInf, -kInf, kNaN}) {
            auto r = j == 0 ? edgeneuro::mearm::base_ramp() : j == 1 ? edgeneuro::mearm::shoulder_ramp()
                   : j == 2 ? edgeneuro::mearm::elbow_ramp() : edgeneuro::mearm::claw_ramp();
            for (int i = 0; i < 30000; ++i) {
                const unsigned out = r.step(target, 0.001f);
                INFO(arm[j].name << " target=" << target);
                REQUIRE(out >= arm[j].lo);
                REQUIRE(out <= arm[j].hi);
            }
        }
    }
}

TEST_CASE("a real-arm ramp does not move faster than a few hundred microseconds per second before it has arrived", "[mearm][ramp]") {
    // the point of the ramp: from rest it must not reach a far target within a second
    auto r = edgeneuro::mearm::base_ramp();
    for (int i = 0; i < 1000; ++i) r.step(2500.0f, 0.001f);      // 1 s toward the far end
    REQUIRE(r.current_us() < 2000u);                             // a jump would be at 2500
}

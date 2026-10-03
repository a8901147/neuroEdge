#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/servo_angle_map.hpp"

using Catch::Approx;
using edgeneuro::ServoAngleMap;

TEST_CASE("ServoAngleMap maps value_min to pulse_min_us exactly", "[control]") {
    ServoAngleMap map(-1.0f, 1.0f, 500u, 2500u);
    REQUIRE(map.pulse_us(-1.0f) == 500u);
}

TEST_CASE("ServoAngleMap maps value_max to pulse_max_us exactly", "[control]") {
    ServoAngleMap map(-1.0f, 1.0f, 500u, 2500u);
    REQUIRE(map.pulse_us(1.0f) == 2500u);
}

TEST_CASE("ServoAngleMap maps the midpoint value to the midpoint pulse", "[control]") {
    ServoAngleMap map(-1.0f, 1.0f, 500u, 2500u);
    REQUIRE(map.pulse_us(0.0f) == 1500u);
}

TEST_CASE("ServoAngleMap clamps a value below value_min to pulse_min_us", "[control]") {
    // A live sensor reading can momentarily exceed its own calibrated
    // range (noise, or a real motion past what calibration captured) --
    // must not extrapolate past the servo's own real safe range.
    ServoAngleMap map(-1.0f, 1.0f, 500u, 2500u);
    REQUIRE(map.pulse_us(-5.0f) == 500u);
}

TEST_CASE("ServoAngleMap clamps a value above value_max to pulse_max_us", "[control]") {
    ServoAngleMap map(-1.0f, 1.0f, 500u, 2500u);
    REQUIRE(map.pulse_us(5.0f) == 2500u);
}

TEST_CASE("ServoAngleMap supports a reversed pulse polarity (larger value -> smaller pulse)", "[control]") {
    // The caller picks which physical end pulse_min_us/pulse_max_us
    // corresponds to -- this must not assume "value increases" implies
    // "pulse increases".
    ServoAngleMap map(-1.0f, 1.0f, 2500u, 500u);
    REQUIRE(map.pulse_us(-1.0f) == 2500u);
    REQUIRE(map.pulse_us(1.0f) == 500u);
    REQUIRE(map.pulse_us(0.0f) == 1500u);
}

TEST_CASE("ServoAngleMap handles a real shoulder_pitch-shaped range", "[control]") {
    // Mirrors run_demo_live.py's SHOULDER_PITCH_RANGE = (-3.0892, 1.0472)
    // mapped onto a placeholder servo range -- exercises a non-symmetric,
    // non-zero-centered real value range, not just a tidy -1..1 example.
    ServoAngleMap map(-3.0892f, 1.0472f, 450u, 2500u);
    REQUIRE(map.pulse_us(-3.0892f) == 450u);
    REQUIRE(map.pulse_us(1.0472f) == 2500u);
    // Baseline (hang-down, value=0.0) should land somewhere strictly
    // between the two ends, not at either extreme.
    const unsigned baseline_pulse = map.pulse_us(0.0f);
    REQUIRE(baseline_pulse > 450u);
    REQUIRE(baseline_pulse < 2500u);
}

TEST_CASE("ServoAngleMap handles a 0..1 grip-shaped range", "[control]") {
    ServoAngleMap map(0.0f, 1.0f, 600u, 2400u);
    REQUIRE(map.pulse_us(0.0f) == 600u);
    REQUIRE(map.pulse_us(1.0f) == 2400u);
}

// ---- degenerate / non-finite configuration (2026-09-26 review) ----
// A map whose value range is empty or reversed-by-mistake, or has a non-finite
// bound, divides by zero (0/0 = NaN) and then hits static_cast<unsigned>(NaN),
// which is undefined behaviour. None of the constants in use today are like
// that, but the class is public: it must fail safe (neutral = range midpoint),
// never with an out-of-range pulse.

#include <cmath>
#include <limits>

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

void require_neutral_and_in_range(const ServoAngleMap& map, unsigned lo, unsigned hi) {
    for (float v : {-10.0f, -1.0f, 0.0f, 0.5f, 1.0f, 10.0f, kNaN, kInf, -kInf}) {
        const unsigned p = map.pulse_us(v);
        INFO("value=" << v);
        REQUIRE(p >= (lo < hi ? lo : hi));
        REQUIRE(p <= (lo < hi ? hi : lo));
    }
}
}  // namespace

TEST_CASE("ServoAngleMap with an empty value range (min == max) fails safe", "[control]") {
    ServoAngleMap map(1.0f, 1.0f, 500u, 2500u);
    require_neutral_and_in_range(map, 500u, 2500u);
    REQUIRE(map.pulse_us(1.0f) == 1500u);
}

TEST_CASE("ServoAngleMap with min > max (misconfigured) fails safe", "[control]") {
    ServoAngleMap map(2.0f, -2.0f, 500u, 2500u);
    require_neutral_and_in_range(map, 500u, 2500u);
    REQUIRE(map.pulse_us(0.0f) == 1500u);
}

TEST_CASE("ServoAngleMap with a non-finite value bound fails safe", "[control]") {
    require_neutral_and_in_range(ServoAngleMap(kNaN, 1.0f, 500u, 2500u), 500u, 2500u);
    require_neutral_and_in_range(ServoAngleMap(-1.0f, kNaN, 500u, 2500u), 500u, 2500u);
    require_neutral_and_in_range(ServoAngleMap(-kInf, kInf, 500u, 2500u), 500u, 2500u);
}

TEST_CASE("ServoAngleMap with equal pulse ends always returns that pulse", "[control]") {
    ServoAngleMap map(-1.0f, 1.0f, 1500u, 1500u);
    for (float v : {-5.0f, 0.0f, 5.0f, kNaN}) REQUIRE(map.pulse_us(v) == 1500u);
}

TEST_CASE("ServoAngleMap with a huge but finite value range never leaves the pulse range", "[control]") {
    // (max - min) overflows float to +inf here
    const float big = std::numeric_limits<float>::max();
    ServoAngleMap map(-big, big, 500u, 2500u);
    require_neutral_and_in_range(map, 500u, 2500u);
}

// The base's comfortable reach (2026-10-03, the user's choice): the base keeps its whole 500..2500 us, but its ends are
// reached at the arm's own comfortable left/right reach (measure_base_reach.py) instead of the calibration's ~+-34 deg,
// so a turn of the arm moves the base less. Without a measured reach: exactly the previous mapping. Same numbers as
// tools/mujoco_bridge/test_mearm_real.py's BaseReachTest (Python is the source of truth).
#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_drive.hpp"

namespace M = edgeneuro::mearm;
namespace R = edgeneuro::mearm::real;
namespace D = edgeneuro::mearm::drive;
namespace C = edgeneuro::mearm::calibration_data;
using V = edgeneuro::mearm::pathb::Vec3;

namespace {
constexpr float kPi = 3.14159265f;
float rad(float deg) { return deg * kPi / 180.0f; }

const M::pathb::Calibration& cal() {
    static M::pathb::Calibration c;
    static const bool ok = M::make_compiled_calibration(c);
    REQUIRE(ok);
    return c;
}
V unit(V v) {
    const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return {v[0] / n, v[1] / n, v[2] / n};
}
// the arm held forward, turned `deg` to the LEFT (a +angle about the HANG axis is a turn to the RIGHT here)
V turned_left(float deg) {
    const V k = unit({C::kHangX, C::kHangY, C::kHangZ});
    const V v{C::kForwardX, C::kForwardY, C::kForwardZ};
    const float a = -rad(deg), c = std::cos(a), s = std::sin(a), d = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
    const V x{k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]};
    V r{};
    for (int i = 0; i < 3; ++i) r[i] = v[i] * c + x[i] * s + k[i] * d * (1 - c);
    float tilt = 0, az = 0;
    REQUIRE(cal().decode(r, tilt, az));
    REQUIRE(std::fabs(az - rad(deg)) < rad(0.5f));
    return r;
}
}  // namespace

TEST_CASE("base reach: without a measured reach the base is exactly as before", "[mearm][base_reach]") {
    const R::BaseReach def = R::default_base_reach(cal());
    for (float deg : {-40.0f, -20.0f, -5.0f, 0.0f, 7.0f, 25.0f, 33.0f, 50.0f}) {
        const V v = turned_left(deg);
        REQUIRE(R::base_pulse(cal(), v, def) == R::base_pulse(cal(), v));
    }
}

TEST_CASE("base reach: the measured reach is where the base hits its ends, halfway is halfway", "[mearm][base_reach]") {
    const R::BaseReach reach{rad(60.0f), rad(-50.0f)};
    auto at = [&](float deg) { return static_cast<long>(R::base_pulse(cal(), turned_left(deg), reach)); };
    REQUIRE(at(0.0f) == static_cast<long>(R::kRestBaseUs));
    REQUIRE(std::labs(at(30.0f) - 2000) <= 3);
    REQUIRE(std::labs(at(-25.0f) - 1000) <= 3);
    REQUIRE(at(65.0f) == 2500);
    REQUIRE(at(-55.0f) == 500);
}

TEST_CASE("base reach: nearly hanging the base stays at rest whatever the reach", "[mearm][base_reach]") {
    // 5 deg off HANG towards the left (the same pose as the Python test: slerp HANG -> left-80 by 5/73)
    const V h = unit({C::kHangX, C::kHangY, C::kHangZ});
    const V l = unit(turned_left(80.0f));
    const float om = std::acos(h[0] * l[0] + h[1] * l[1] + h[2] * l[2]);
    const float t = 5.0f / 73.0f;
    V v{};
    for (int i = 0; i < 3; ++i) v[i] = (std::sin((1 - t) * om) * h[i] + std::sin(t * om) * l[i]) / std::sin(om);
    REQUIRE(R::base_pulse(cal(), v, R::BaseReach{rad(30.0f), rad(-30.0f)}) == R::kRestBaseUs);
}

TEST_CASE("base reach: the compiled-in reach is used when measured, the default otherwise", "[mearm][base_reach]") {
    R::BaseReach r{};
    REQUIRE(M::make_compiled_base_reach(cal(), r));
    const R::BaseReach def = R::default_base_reach(cal());
    if (!C::kHasBaseReach) {
        REQUIRE((r.left == def.left && r.right == def.right));
    } else {
        REQUIRE((r.left > 0.1f && r.right < -0.1f));
    }
}

TEST_CASE("base reach: a reach on one side only is refused", "[mearm][base_reach]") {
    R::BaseReach r{};
    REQUIRE_FALSE(R::make_base_reach(cal(), turned_left(40.0f), turned_left(20.0f), r));
    REQUIRE(R::make_base_reach(cal(), turned_left(40.0f), turned_left(-35.0f), r));
    REQUIRE(std::fabs(r.left - rad(40.0f)) < rad(0.5f));
    REQUIRE(std::fabs(r.right - rad(-35.0f)) < rad(0.5f));
}

TEST_CASE("base reach: drive::command uses the reach it is given", "[mearm][base_reach]") {
    const R::BaseReach wide{rad(60.0f), rad(-60.0f)};
    const V v = turned_left(30.0f);
    REQUIRE(D::command(&cal(), v, C::kZeroElbow, 0.0f, true, &wide).base == R::base_pulse(cal(), v, wide));
    REQUIRE(D::command(&cal(), v, C::kZeroElbow, 0.0f, true).base == R::base_pulse(cal(), v));
}

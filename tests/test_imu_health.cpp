// edgeneuro::ImuHealth: the firmware-side sensor check that keeps the REAL arm from following a failed IMU (2026-09-28;
// the failures behind it: SESSION_LOG 2026-09-27 -- the upper arm dropping out and repeating its last value, the forearm
// stuck at (1.999939, 0, 0) while its reads still "completed"). Updated once per servo cycle (100 Hz).
#include <cmath>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/imu_health.hpp"

using edgeneuro::ImuHealth;
using V = edgeneuro::ImuHealth::Vec3;

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
const V kHang{0.998f, 0.004f, 0.243f};           // real 9/28 reading, |a| = 1.03 g
const V kFore{0.95f, 0.05f, 0.30f};

V noisy(const V& v, int i) {                      // deterministic sensor-like noise (+-0.003 g)
    const float e = 0.003f * static_cast<float>((i * 37) % 7 - 3) / 3.0f;
    return {v[0] + e, v[1] - e, v[2] + 0.5f * e};
}

void feed(ImuHealth& h, int n, bool freeze_upper = false, bool freeze_fore = false) {
    for (int i = 0; i < n; ++i) {
        h.update(freeze_upper ? kHang : noisy(kHang, i), freeze_fore ? kFore : noisy(kFore, i + 3));
    }
}
}  // namespace

TEST_CASE("live noisy readings are healthy", "[health]") {
    ImuHealth h;
    feed(h, 100);
    REQUIRE(h.ok());
}

TEST_CASE("a new monitor is not healthy until it has seen live data", "[health]") {
    ImuHealth h;
    REQUIRE_FALSE(h.ok());
    feed(h, 3);
    REQUIRE(h.ok());                               // three changing, plausible readings are enough to drive
}

TEST_CASE("the real stuck forearm (1.999939, 0, 0) is a fault at once", "[health]") {
    ImuHealth h;
    feed(h, 50);
    h.update(noisy(kHang, 1), V{1.999939f, 0.0f, 0.0f});
    REQUIRE_FALSE(h.ok());
    REQUIRE(h.forearm_fault());
    REQUIRE_FALSE(h.upper_arm_fault());
}

TEST_CASE("the real 0.21 g dropped-out reading is a fault at once", "[health]") {
    ImuHealth h;
    feed(h, 50);
    h.update(V{0.179f, 0.057f, 0.093f}, noisy(kFore, 1));
    REQUIRE(h.upper_arm_fault());
}

TEST_CASE("the real 2.2 g dropped-out reading is caught because it repeats, within 0.3 s", "[health]") {
    // one 2.2 g reading could be a fast swing (no axis at full scale, below 3 g), so it is not an instant fault;
    // what gave the real one away is that it never changed
    ImuHealth h;
    feed(h, 50);
    const V bad{-1.687f, -0.988f, -0.988f};
    int updates = 0;
    while (h.ok() && updates < 100) {
        h.update(bad, noisy(kFore, updates));
        ++updates;
    }
    REQUIRE(h.upper_arm_fault());
    REQUIRE(updates <= ImuHealth::kFrozenUpdates);
}

TEST_CASE("a value that stops changing for 30 updates (0.3 s) is a fault, a short repeat is not", "[health]") {
    ImuHealth h;
    feed(h, 50);
    for (int i = 0; i < 29; ++i) h.update(kHang, noisy(kFore, i));    // perfectly plausible, just never changes
    REQUIRE(h.ok());
    h.update(kHang, noisy(kFore, 99));
    REQUIRE_FALSE(h.ok());
    REQUIRE(h.upper_arm_fault());
}

TEST_CASE("non-finite or all-zero readings are faults", "[health]") {
    for (V bad : {V{kNaN, 0.0f, 1.0f}, V{0.0f, 0.0f, 0.0f}}) {
        ImuHealth h;
        feed(h, 50);
        h.update(noisy(kHang, 1), bad);
        REQUIRE(h.forearm_fault());
    }
}

TEST_CASE("it recovers by itself once live data returns", "[health]") {
    ImuHealth h;
    feed(h, 50);
    h.update(noisy(kHang, 1), V{1.999939f, 0.0f, 0.0f});
    REQUIRE_FALSE(h.ok());
    feed(h, 3);
    REQUIRE(h.ok());
}

TEST_CASE("a sensor declared optional is never a fault", "[health]") {
    ImuHealth h(/*check_upper_arm=*/true, /*check_forearm=*/false);
    for (int i = 0; i < 50; ++i) h.update(noisy(kHang, i), V{1.999939f, 0.0f, 0.0f});
    REQUIRE(h.ok());
}

// Vec3OneEuro: the 1-euro filter (Casiez, Roussel, Vogel, CHI 2012) on an IMU's raw accel vector, for the real MEArm's
// servo path (2026-10-02: the base swung back and forth on tiny arm motions; a fixed EMA has to trade jitter at rest
// against lag in motion, the 1-euro filter's cutoff rises with speed instead). These tests use synthetic signals and
// check the filter's PROPERTIES with headroom; its two parameters are to be chosen from real captured arm data.
#include <cmath>
#include <limits>
#include <random>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/filters/vec3_one_euro.hpp"

using edgeneuro::Vec3OneEuro;

namespace {
constexpr float kDt = 0.01f;                       // the firmware's servo cadence (every 10th 1 kHz tick)
constexpr float kPi = 3.14159265f;
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// a unit gravity vector tilted by `a` radians in the x-z plane
Vec3OneEuro::Vec tilted(float a) { return {std::sin(a), 0.0f, std::cos(a)}; }
}  // namespace

TEST_CASE("1-euro: the first real reading is taken as-is", "[filters][one_euro]") {
    Vec3OneEuro f(1.0f, 0.5f);
    const auto out = f.update(0.99f, 0.05f, 0.26f, kDt);
    REQUIRE((out.x == 0.99f && out.y == 0.05f && out.z == 0.26f));
}

TEST_CASE("1-euro: the firmware's all-zero 'no reading yet' vector passes through and does not seed it",
          "[filters][one_euro]") {
    Vec3OneEuro f(1.0f, 0.5f);
    const auto z = f.update(0.0f, 0.0f, 0.0f, kDt);
    REQUIRE((z.x == 0.0f && z.y == 0.0f && z.z == 0.0f));
    REQUIRE_FALSE(f.initialized());
    REQUIRE(f.update(0.0f, 0.0f, 1.0f, kDt).z == 1.0f);
}

TEST_CASE("1-euro: on a still arm, sensor noise shrinks a lot", "[filters][one_euro]") {
    std::mt19937 rng(7);
    std::normal_distribution<float> noise(0.0f, 0.01f);
    Vec3OneEuro f(1.0f, 0.5f);
    double raw_sq = 0.0, out_sq = 0.0;
    int n = 0;
    for (int i = 0; i < 3000; ++i) {
        const float nx = noise(rng), ny = noise(rng), nz = noise(rng);
        const auto out = f.update(0.99f + nx, 0.05f + ny, 0.26f + nz, kDt);
        if (i >= 300) {
            raw_sq += static_cast<double>(nx) * nx;
            out_sq += static_cast<double>(out.x - 0.99f) * (out.x - 0.99f);
            ++n;
        }
    }
    REQUIRE(std::sqrt(raw_sq / n) > 4.0 * std::sqrt(out_sq / n));
}

TEST_CASE("1-euro: in fast motion it lags far less than a fixed low-pass that is just as smooth at rest",
          "[filters][one_euro]") {
    // the same min cutoff; beta = 0 IS that fixed low-pass. Arm rotating at 2 rad/s (an ordinary demo motion).
    Vec3OneEuro adaptive(1.0f, 0.5f), fixed(1.0f, 0.0f);
    float worst_adaptive = 0.0f, worst_fixed = 0.0f;
    for (int i = 0; i <= 50; ++i) {
        const auto v = tilted(2.0f * kDt * static_cast<float>(i));
        const auto a = adaptive.update(v.x, v.y, v.z, kDt);
        const auto b = fixed.update(v.x, v.y, v.z, kDt);
        if (i >= 20) {
            worst_adaptive = std::fmax(worst_adaptive, std::fabs(a.x - v.x));
            worst_fixed = std::fmax(worst_fixed, std::fabs(b.x - v.x));
        }
    }
    REQUIRE(worst_fixed > 3.0f * worst_adaptive);
}

TEST_CASE("1-euro: with beta = 0 it is exactly a first-order low-pass at the min cutoff", "[filters][one_euro]") {
    Vec3OneEuro f(2.0f, 0.0f);
    f.update(1.0f, 0.0f, 0.0f, kDt);
    const float tau = 1.0f / (2.0f * kPi * 2.0f);
    const float alpha = 1.0f / (1.0f + tau / kDt);
    const auto out = f.update(0.0f, 0.0f, 1.0f, kDt);
    REQUIRE(std::fabs(out.z - alpha) < 1e-6f);
    REQUIRE(std::fabs(out.x - (1.0f - alpha)) < 1e-6f);
}

TEST_CASE("1-euro: one speed for the whole vector -- fast motion on one axis relaxes the filter on all of them",
          "[filters][one_euro]") {
    // per-axis cutoffs would distort the direction (the base's azimuth); measured by how fast y follows a step while
    // x is moving fast vs while x is still
    auto y_after_step = [](bool x_moving) {
        Vec3OneEuro f(1.0f, 0.5f);
        f.update(1.0f, 0.0f, 0.0f, kDt);
        Vec3OneEuro::Vec out{};
        for (int i = 1; i <= 5; ++i) {
            const float x = x_moving ? 1.0f - 0.03f * static_cast<float>(i) : 1.0f;
            out = f.update(x, 0.1f, 0.0f, kDt);
        }
        return out.y;
    };
    REQUIRE(y_after_step(true) > y_after_step(false));
}

TEST_CASE("1-euro: a non-finite reading or a non-positive dt is ignored", "[filters][one_euro]") {
    Vec3OneEuro f(1.0f, 0.5f);
    f.update(0.0f, 0.0f, 1.0f, kDt);
    for (const float bad : {kNaN, kInf, -kInf}) {
        const auto out = f.update(bad, 0.0f, 1.0f, kDt);
        REQUIRE((out.x == 0.0f && out.z == 1.0f));
    }
    for (const float dt : {0.0f, -0.01f, kNaN}) {
        const auto out = f.update(1.0f, 0.0f, 0.0f, dt);
        REQUIRE((out.x == 0.0f && out.z == 1.0f));
    }
}

TEST_CASE("1-euro: a constant input stays exactly constant", "[filters][one_euro]") {
    Vec3OneEuro f(1.0f, 0.5f);
    Vec3OneEuro::Vec out{};
    for (int i = 0; i < 1000; ++i) out = f.update(0.3f, -0.4f, 0.85f, kDt);
    REQUIRE((out.x == 0.3f && out.y == -0.4f && out.z == 0.85f));
}

TEST_CASE("1-euro: the speed counts motion on every axis, not just one", "[filters][one_euro]") {
    // fast motion on z alone must relax the filter on x too (the shared-speed test above only moves x)
    auto x_after_step = [](bool z_moving) {
        Vec3OneEuro f(1.0f, 0.5f);
        f.update(0.0f, 0.0f, 1.0f, kDt);
        Vec3OneEuro::Vec out{};
        for (int i = 1; i <= 5; ++i) {
            const float z = z_moving ? 1.0f - 0.03f * static_cast<float>(i) : 1.0f;
            out = f.update(0.1f, 0.0f, z, kDt);
        }
        return out.x;
    };
    REQUIRE(x_after_step(true) > x_after_step(false));
}

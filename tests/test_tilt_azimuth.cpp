// Verifies the tilt/azimuth replacement for complementary_filter.hpp's
// accel_pitch_angle, which a real hardware session (2026-09-04, see git
// history / PRD.md) showed folding back past +-90deg -- two genuinely
// different real rotations (e.g. a forward-raise and a backward-extension)
// could decode to the identical pitch. Tested purely against known physics
// (synthetic unit vectors at known tilt/azimuth), the same
// sensor-format-independent way test_complementary_filter.cpp is -- see
// that file's own top-of-file rationale.
//
// Directly answers "does this cover hang-down / forward-raise / front-left
// raise / front-right raise, and everything past 90deg that broke the old
// formula" with a real, checkable test rather than an assertion in
// conversation: FULL_ROM_SWEEP below sweeps tilt from 0 to 170deg (short of
// the one physically-unreachable singularity, a full fold-back at 180deg)
// at each of six azimuth directions spanning the full circle, and checks
// exact recovery at every single point.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>

#include "edgeneuro/fusion/tilt_azimuth.hpp"

using Catch::Approx;
using edgeneuro::orthonormal_basis_perpendicular_to;
using edgeneuro::tilt_azimuth;

namespace {

struct Vec3 {
    float x, y, z;
};

// Builds the accel vector that a real sensor would read at the given
// tilt/azimuth (radians) away from `ref`, using `ref`'s own perpendicular
// basis -- the exact inverse of what tilt_azimuth() computes, so a round
// trip (synthesize -> decode) must recover the same (tilt, azimuth) if the
// implementation is correct. Standard spherical-to-Cartesian construction:
// ref*cos(tilt) + (u*cos(azimuth) + v*sin(azimuth))*sin(tilt).
Vec3 synthesize(const Vec3& ref, const Vec3& u, const Vec3& v, float tilt, float azimuth) {
    const float s = std::sin(tilt);
    const float c = std::cos(tilt);
    const float cu = std::cos(azimuth);
    const float sv = std::sin(azimuth);
    return Vec3{
        ref.x * c + (u.x * cu + v.x * sv) * s,
        ref.y * c + (u.y * cu + v.y * sv) * s,
        ref.z * c + (u.z * cu + v.z * sv) * s,
    };
}

} // namespace

TEST_CASE("orthonormal_basis_perpendicular_to produces a valid right-handed basis", "[fusion][tilt_azimuth]") {
    // A non-axis-aligned reference, on purpose -- proves this isn't only
    // correct for the convenient (0,0,1) case. Roughly matches the real
    // mount's own rest-pose magnitude ordering (largest on one axis,
    // moderate on another, small on the third), not copied from any
    // specific captured session's exact numbers.
    Vec3 ref{0.6f, -0.3f, 0.74f};
    // Normalize (inputs to tilt_azimuth must be unit vectors).
    float mag = std::sqrt(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z);
    ref = {ref.x / mag, ref.y / mag, ref.z / mag};

    Vec3 u{}, v{};
    orthonormal_basis_perpendicular_to(ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);

    auto dot = [](const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
    auto norm = [&](const Vec3& a) { return std::sqrt(dot(a, a)); };

    REQUIRE(norm(u) == Approx(1.0f).margin(1e-5));
    REQUIRE(norm(v) == Approx(1.0f).margin(1e-5));
    REQUIRE(dot(u, ref) == Approx(0.0f).margin(1e-5));
    REQUIRE(dot(v, ref) == Approx(0.0f).margin(1e-5));
    REQUIRE(dot(u, v) == Approx(0.0f).margin(1e-5));
}

TEST_CASE("tilt_azimuth: hang-down, forward, front-left, front-right all decode correctly", "[fusion][tilt_azimuth]") {
    // Same non-trivial reference as above.
    Vec3 ref{0.6f, -0.3f, 0.74f};
    float mag = std::sqrt(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z);
    ref = {ref.x / mag, ref.y / mag, ref.z / mag};
    Vec3 u{}, v{};
    orthonormal_basis_perpendicular_to(ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);

    constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;

    SECTION("hang-down: exactly at rest, tilt=0") {
        auto result = tilt_azimuth(ref.x, ref.y, ref.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
        REQUIRE(result.tilt == Approx(0.0f).margin(1e-5));
    }

    SECTION("forward raise: azimuth=0 (toward u), a real ~120deg flexion") {
        const float true_tilt = 120.0f * kDeg;
        const float true_azimuth = 0.0f;
        Vec3 a = synthesize(ref, u, v, true_tilt, true_azimuth);
        auto result = tilt_azimuth(a.x, a.y, a.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
        REQUIRE(result.tilt == Approx(true_tilt).margin(1e-4));
        REQUIRE(result.azimuth == Approx(true_azimuth).margin(1e-4));
    }

    SECTION("front-left raise: azimuth=+45deg, a real ~100deg elevation") {
        const float true_tilt = 100.0f * kDeg;
        const float true_azimuth = 45.0f * kDeg;
        Vec3 a = synthesize(ref, u, v, true_tilt, true_azimuth);
        auto result = tilt_azimuth(a.x, a.y, a.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
        REQUIRE(result.tilt == Approx(true_tilt).margin(1e-4));
        REQUIRE(result.azimuth == Approx(true_azimuth).margin(1e-4));
    }

    SECTION("front-right raise: azimuth=-45deg, a real ~100deg elevation") {
        const float true_tilt = 100.0f * kDeg;
        const float true_azimuth = -45.0f * kDeg;
        Vec3 a = synthesize(ref, u, v, true_tilt, true_azimuth);
        auto result = tilt_azimuth(a.x, a.y, a.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
        REQUIRE(result.tilt == Approx(true_tilt).margin(1e-4));
        REQUIRE(result.azimuth == Approx(true_azimuth).margin(1e-4));
    }

    SECTION("forward raise vs backward extension (azimuth=180deg) are NOT confused -- the exact 2026-09-04 bug") {
        Vec3 forward = synthesize(ref, u, v, 110.0f * kDeg, 0.0f);
        Vec3 backward = synthesize(ref, u, v, 90.0f * kDeg, 180.0f * kDeg);
        auto rf = tilt_azimuth(forward.x, forward.y, forward.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
        auto rb = tilt_azimuth(backward.x, backward.y, backward.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
        // Old formula's exact failure mode: these decoded to the SAME
        // pitch. Here, tilt alone doesn't distinguish direction (by
        // design -- tilt is magnitude-only), but azimuth must clearly
        // separate them.
        REQUIRE(std::abs(rf.azimuth - rb.azimuth) > (std::numbers::pi_v<float> / 2.0f));
    }
}

TEST_CASE("tilt_azimuth full ROM sweep: no ambiguity anywhere in a real shoulder's reachable range", "[fusion][tilt_azimuth]") {
    Vec3 ref{0.6f, -0.3f, 0.74f};
    float mag = std::sqrt(ref.x * ref.x + ref.y * ref.y + ref.z * ref.z);
    ref = {ref.x / mag, ref.y / mag, ref.z / mag};
    Vec3 u{}, v{};
    orthonormal_basis_perpendicular_to(ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);

    constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
    // Six azimuth directions spanning the full circle (forward, front-left,
    // left, back, right, front-right) -- not just the four named in
    // conversation, to also cover pure left/right and backward.
    const float azimuths_deg[] = {0.0f, 45.0f, 90.0f, 180.0f, -90.0f, -45.0f};

    for (float az_deg : azimuths_deg) {
        const float true_azimuth = az_deg * kDeg;
        // 0..170deg: real shoulder ROM (flexion up to ~180deg, this
        // project's own anatomical reference elsewhere) short of the one
        // genuine, physically-unreachable singularity at a full 180deg
        // fold-back onto the reference direction itself.
        for (int tilt_deg = 0; tilt_deg <= 170; tilt_deg += 5) {
            const float true_tilt = static_cast<float>(tilt_deg) * kDeg;
            Vec3 a = synthesize(ref, u, v, true_tilt, true_azimuth);
            auto result = tilt_azimuth(a.x, a.y, a.z, ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);

            REQUIRE(result.tilt == Approx(true_tilt).margin(1e-3));
            if (tilt_deg > 0) { // azimuth undefined at tilt=0, by design -- see header comment
                // Wrapped comparison: +180deg and -180deg are the same
                // direction (atan2's one unavoidable branch point, same as
                // +-180deg longitude being the same meridian) -- a plain
                // Approx would spuriously fail exactly at that one point.
                float diff = result.azimuth - true_azimuth;
                const float two_pi = 2.0f * std::numbers::pi_v<float>;
                diff = std::fmod(diff + std::numbers::pi_v<float>, two_pi);
                if (diff < 0) diff += two_pi;
                diff -= std::numbers::pi_v<float>;
                REQUIRE(diff == Approx(0.0f).margin(1e-3));
            }
        }
    }
}

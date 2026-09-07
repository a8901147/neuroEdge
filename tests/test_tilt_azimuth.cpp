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
using edgeneuro::make_oblique_basis;
using edgeneuro::oblique_decompose;
using edgeneuro::oblique_decompose_scaled;
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
        // 1e-3, not 1e-5 (2026-09-07): acos'(x) = -1/sqrt(1-x^2) blows up as
        // x->1, so this exact-self-dot-product case (dot(ref,ref) should be
        // 1.0 but float32 arithmetic lands a few ULPs off) amplifies a
        // sub-epsilon input difference into a real, platform-dependent
        // output difference -- CI (x86_64 Linux/GCC) measured 0.00035rad
        // (~0.02deg) here against a margin tuned only against one platform
        // (arm64 macOS/Clang), which never exercised this. 0.02deg is
        // physically meaningless noise, not an algorithm bug -- matches
        // this file's own 1e-3 convention for its other hard-to-pin-exactly
        // cases (see line ~161).
        REQUIRE(result.tilt == Approx(0.0f).margin(1e-3));
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

// oblique_decompose replaces tilt_azimuth's azimuth for driving two
// independent joint controls (e.g. MuJoCo's shoulder_pitch/shoulder_roll):
// see this file's include for why assuming fwd/abd are perpendicular
// doesn't hold for a real shoulder. These fixtures are deliberately NOT
// perpendicular (fwd at tilt=70deg/azimuth=0deg, abd at tilt=80deg/
// azimuth=25deg -- only 25deg apart, similar order of magnitude to the
// ~29deg found on real hardware) to prove the math handles an oblique
// basis correctly, not just the convenient orthogonal case.
TEST_CASE("oblique_decompose recovers exact coordinates in a non-orthogonal basis", "[fusion][tilt_azimuth]") {
    const Vec3 ref{0.0f, 0.0f, 1.0f};
    Vec3 u{}, v{};
    orthonormal_basis_perpendicular_to(ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);

    constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
    const Vec3 fwd_ref = synthesize(ref, u, v, 70.0f * kDeg, 0.0f * kDeg);
    const Vec3 abd_ref = synthesize(ref, u, v, 80.0f * kDeg, 25.0f * kDeg);

    auto basis = make_oblique_basis(ref.x, ref.y, ref.z, fwd_ref.x, fwd_ref.y, fwd_ref.z,
                                     abd_ref.x, abd_ref.y, abd_ref.z);

    // Anchor points: the calibration readings themselves must decode to
    // exactly (1,0) and (0,1), and the reference itself to (0,0) -- these
    // are the defining properties of the basis, not just plausible values.
    auto at_ref = oblique_decompose(basis, ref.x, ref.y, ref.z);
    REQUIRE(at_ref.fwd == Approx(0.0f).margin(1e-5));
    REQUIRE(at_ref.abd == Approx(0.0f).margin(1e-5));

    auto at_fwd = oblique_decompose(basis, fwd_ref.x, fwd_ref.y, fwd_ref.z);
    REQUIRE(at_fwd.fwd == Approx(1.0f).margin(1e-4));
    REQUIRE(at_fwd.abd == Approx(0.0f).margin(1e-4));

    auto at_abd = oblique_decompose(basis, abd_ref.x, abd_ref.y, abd_ref.z);
    REQUIRE(at_abd.fwd == Approx(0.0f).margin(1e-4));
    REQUIRE(at_abd.abd == Approx(1.0f).margin(1e-4));

    // A non-anchor combination (0.3*pf + 0.7*pa, offset back onto the unit
    // sphere by adding ref) must recover exactly (0.3, 0.7) -- verifies the
    // Gram-matrix solve itself, not just the two trivial anchor cases.
    // (Expected value cross-checked independently in numpy at double
    // precision; not hand-derived.)
    const Vec3 v_mid{-0.291338419f, 0.906685041f, 1.0f};
    auto at_mid = oblique_decompose(basis, v_mid.x, v_mid.y, v_mid.z);
    REQUIRE(at_mid.fwd == Approx(0.3f).margin(1e-4));
    REQUIRE(at_mid.abd == Approx(0.7f).margin(1e-4));
}

// Sanity-checks oblique_decompose against the real shoulder capture this
// was built from (tools/mujoco_bridge/raw_imu_calibration.json, 6-pose x
// 5-repeat capture averaged per pose, 2026-09-05 -- see PRD.md's Session
// Handoff). Wide margins on purpose: unlike the synthetic test above, these
// aren't exact by construction -- BACKWARD_EXTENSION and ADDUCTION_RIGHT
// were never part of building the basis, so what matters here is the
// SIGN and rough magnitude (does "backward" decode as forward-negative,
// does "adduction" decode as abduction-negative), documenting the actual
// measured cross-talk rather than an idealized expectation.
TEST_CASE("oblique_decompose against real captured shoulder data: signs match anatomy despite cross-talk", "[fusion][tilt_azimuth]") {
    const Vec3 ref{0.96756683f, -0.24536111f, -0.06010292f};
    const Vec3 fwd_ref{0.23614663f, -0.53617526f, 0.81040166f};
    const Vec3 abd_ref{0.11104646f, -0.91944181f, 0.37722067f};
    auto basis = make_oblique_basis(ref.x, ref.y, ref.z, fwd_ref.x, fwd_ref.y, fwd_ref.z,
                                     abd_ref.x, abd_ref.y, abd_ref.z);

    auto at_fwd = oblique_decompose(basis, fwd_ref.x, fwd_ref.y, fwd_ref.z);
    REQUIRE(at_fwd.fwd == Approx(1.0f).margin(1e-3));
    REQUIRE(at_fwd.abd == Approx(0.0f).margin(1e-3));

    auto at_abd = oblique_decompose(basis, abd_ref.x, abd_ref.y, abd_ref.z);
    REQUIRE(at_abd.fwd == Approx(0.0f).margin(1e-3));
    REQUIRE(at_abd.abd == Approx(1.0f).margin(1e-3));

    // BACKWARD_EXTENSION: real captured direction, roughly opposite
    // FORWARD_RAISE -- must come out fwd-negative. It also picks up a
    // large abd component (documented cross-talk, not a bug).
    const Vec3 backward{0.62289630f, -0.43870541f, -0.64771734f};
    auto at_backward = oblique_decompose(basis, backward.x, backward.y, backward.z);
    REQUIRE(at_backward.fwd < 0.0f);

    // ADDUCTION_RIGHT: real captured direction -- must come out
    // abd-negative (opposing ABDUCTION_LEFT), matching the azimuth finding
    // that its direction leans toward FORWARD_RAISE rather than being a
    // clean opposite of ABDUCTION_LEFT.
    const Vec3 adduction{0.76694865f, -0.38367021f, 0.51438016f};
    auto at_adduction = oblique_decompose(basis, adduction.x, adduction.y, adduction.z);
    REQUIRE(at_adduction.abd < 0.0f);
    REQUIRE(at_adduction.fwd > 0.0f);
}

// oblique_decompose_scaled fixes oblique_decompose's real overshoot problem
// (PRD.md 2026-09-04): scaling the raw coefficient by its calibration
// pose's own (large) tilt can more than double the real angle for an
// off-axis direction. This uses the same non-orthogonal synthetic fixture
// as the anchor test above (fwd at tilt=70deg/azimuth=0deg, abd at
// tilt=80deg/azimuth=25deg) so both functions are checked against an
// identical basis.
TEST_CASE("oblique_decompose_scaled preserves the real tilt magnitude", "[fusion][tilt_azimuth]") {
    const Vec3 ref{0.0f, 0.0f, 1.0f};
    Vec3 u{}, v{};
    orthonormal_basis_perpendicular_to(ref.x, ref.y, ref.z, u.x, u.y, u.z, v.x, v.y, v.z);
    constexpr float kDeg = std::numbers::pi_v<float> / 180.0f;
    const Vec3 fwd_ref = synthesize(ref, u, v, 70.0f * kDeg, 0.0f * kDeg);
    const Vec3 abd_ref = synthesize(ref, u, v, 80.0f * kDeg, 25.0f * kDeg);
    auto basis = make_oblique_basis(ref.x, ref.y, ref.z, fwd_ref.x, fwd_ref.y, fwd_ref.z,
                                     abd_ref.x, abd_ref.y, abd_ref.z);

    // Anchors: exact at ref (0,0), and exact at each calibration pose
    // (recovers that pose's own tilt on its own axis, 0 on the other --
    // same properties oblique_decompose has, now also true of the scaled
    // version).
    auto at_ref = oblique_decompose_scaled(basis, ref.x, ref.y, ref.z);
    REQUIRE(at_ref.fwd == Approx(0.0f).margin(1e-5));
    REQUIRE(at_ref.abd == Approx(0.0f).margin(1e-5));

    auto at_fwd = oblique_decompose_scaled(basis, fwd_ref.x, fwd_ref.y, fwd_ref.z);
    REQUIRE(at_fwd.fwd == Approx(70.0f * kDeg).margin(1e-4));
    REQUIRE(at_fwd.abd == Approx(0.0f).margin(1e-4));

    auto at_abd = oblique_decompose_scaled(basis, abd_ref.x, abd_ref.y, abd_ref.z);
    REQUIRE(at_abd.fwd == Approx(0.0f).margin(1e-4));
    REQUIRE(at_abd.abd == Approx(80.0f * kDeg).margin(1e-4));

    // Off-axis direction (partway between fwd and abd, at a real tilt much
    // smaller than either calibration pose's own tilt): the defining
    // property is that the OUTPUT'S total magnitude must equal the real
    // tilt exactly, unlike raw oblique_decompose scaled by a fixed
    // per-axis constant, which can overshoot it.
    const Vec3 off_axis = synthesize(ref, u, v, 20.0f * kDeg, 40.0f * kDeg);
    auto at_off = oblique_decompose_scaled(basis, off_axis.x, off_axis.y, off_axis.z);
    const float recovered_mag = std::sqrt(at_off.fwd * at_off.fwd + at_off.abd * at_off.abd);
    REQUIRE(recovered_mag == Approx(20.0f * kDeg).margin(1e-4));
}

// Regression check against the real ELBOW_FLEXION overshoot this function
// was built to fix (PRD.md 2026-09-04, first found against an earlier
// single-shot capture where a real ~16.2deg shoulder drift blew up to a
// ~35deg pitch_equiv under plain oblique_decompose scaled by the
// calibration poses' own tilt). Same real calibration basis as
// src/mujoco_bridge_demo.cpp's hardcoded constants (tools/mujoco_bridge/
// raw_imu_calibration.json, 6-pose x 5-repeat capture, 2026-09-05) --
// expected values cross-checked independently in numpy, not hand-derived.
// This capture's own ELBOW_FLEXION drift is smaller (~6.4deg, real
// session-to-session variation in how still the upper arm was held) --
// the property under test is still "output magnitude == real raw tilt,
// not inflated", just at this session's own real numbers.
TEST_CASE("oblique_decompose_scaled against real ELBOW_FLEXION data: no more overshoot", "[fusion][tilt_azimuth]") {
    const Vec3 ref{0.96756683f, -0.24536111f, -0.06010292f};
    const Vec3 fwd_ref{0.23614663f, -0.53617526f, 0.81040166f};
    const Vec3 abd_ref{0.11104646f, -0.91944181f, 0.37722067f};
    auto basis = make_oblique_basis(ref.x, ref.y, ref.z, fwd_ref.x, fwd_ref.y, fwd_ref.z,
                                     abd_ref.x, abd_ref.y, abd_ref.z);

    // Raw (unnormalized) ELBOW_FLEXION shoulder reading -- oblique_decompose_scaled
    // normalizes internally, same convention as tilt_azimuth().
    const Vec3 elbow_flexion_raw{0.995489f, -0.140029f, -0.086466f};
    auto scaled = oblique_decompose_scaled(basis, elbow_flexion_raw.x, elbow_flexion_raw.y, elbow_flexion_raw.z);
    REQUIRE(scaled.fwd == Approx(0.02821f).margin(2e-3));
    REQUIRE(scaled.abd == Approx(-0.10767f).margin(2e-3));

    const float mag = std::sqrt(scaled.fwd * scaled.fwd + scaled.abd * scaled.abd);
    REQUIRE(mag == Approx(0.11131f).margin(2e-3)); // the real raw tilt -- must not be exceeded
    REQUIRE(mag < 0.2f); // well under what an unscaled oblique_decompose overshoot would give
}

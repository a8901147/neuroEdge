#pragma once

#include <cmath>

namespace edgeneuro {

// Replaces a fixed-world-frame Euler decomposition (e.g.
// atan2(-accel_x, sqrt(accel_y^2+accel_z^2)), see complementary_filter.hpp's
// accel_pitch_angle) with one centered on wherever the sensor's REST pose
// actually is. A 2D coordinate chart on a sphere always has some
// unavoidable singularity (a real accelerometer's gravity reading only
// ever traces a sphere) -- the point of centering on the rest reading
// isn't to eliminate that singularity, it's to PLACE it somewhere
// anatomically unreachable (opposite rest -- a shoulder folded completely
// backward on itself) instead of wherever a fixed-frame formula happens to
// land, which a real mount can sit uncomfortably close to. See git
// history / PRD.md 2026-09-04 for the real hardware session (a forward-
// raise and backward-extension decoding to the wrong relative sign) this
// was found from, and the biomechanics literature (ISB's YXY Euler
// sequence for the shoulder gimbal-locking at 90deg elevation; the
// published "Tilt-and-Torsion" alternative this mirrors the tilt/azimuth
// half of) that independently arrives at the same shape of fix.
//
// tilt: angle between the current accel reading and the reference
// (rest) accel reading, in [0, pi] radians. 0 = exactly at rest;
// increases moving away from rest in ANY direction. No singularity
// anywhere in a real shoulder's reachable range (would only reach pi at a
// physically impossible full fold-back).
//
// azimuth: which direction (around the reference vector) the tilt is
// toward, in (-pi, pi]. 0 = toward basis_u; +pi/2 = toward basis_v.
// Undefined (returned as 0 by convention, not NaN) only exactly at
// tilt=0, same as compass bearing being meaningless exactly at a pole --
// not a practical problem since "which direction" has no meaning when
// there's no tilt to have a direction.
//
// All inputs must be pre-normalized (unit length) by the caller.
// basis_u/basis_v must be perpendicular to `ref` and to each other (see
// orthonormal_basis_perpendicular_to below to construct them from `ref`
// alone) -- not verified here (hot-path: no allocation/exception budget
// for a runtime check); construct them once at calibration time from the
// same reading used as `ref`, not per-sample.
template <typename ValueType>
struct TiltAzimuth {
    ValueType tilt;
    ValueType azimuth;
};

template <typename ValueType>
TiltAzimuth<ValueType> tilt_azimuth(
    ValueType accel_x, ValueType accel_y, ValueType accel_z,
    ValueType ref_x, ValueType ref_y, ValueType ref_z,
    ValueType u_x, ValueType u_y, ValueType u_z,
    ValueType v_x, ValueType v_y, ValueType v_z) noexcept {
    ValueType dot_ref = accel_x * ref_x + accel_y * ref_y + accel_z * ref_z;
    // Clamp before acos: the dot-product identity can round to just past
    // +-1 in floating point even for exactly-aligned vectors (same
    // reasoning as phase3_control_loop_main.cpp's elbow_bend clamp), and
    // acos() of anything outside [-1,1] is NaN, not a clamped boundary
    // value.
    if (dot_ref > ValueType{1}) dot_ref = ValueType{1};
    if (dot_ref < ValueType{-1}) dot_ref = ValueType{-1};
    const ValueType tilt = std::acos(dot_ref);

    const ValueType proj_u = accel_x * u_x + accel_y * u_y + accel_z * u_z;
    const ValueType proj_v = accel_x * v_x + accel_y * v_y + accel_z * v_z;
    const ValueType azimuth = std::atan2(proj_v, proj_u);

    return TiltAzimuth<ValueType>{tilt, azimuth};
}

// Builds an orthonormal basis (out_u, out_v) spanning the plane
// perpendicular to a unit reference vector `ref`, so tilt_azimuth above can
// be called with nothing more than the rest-pose reading itself -- no
// extra calibration motion needed. Picks (0,0,1) as the seed to cross
// against, unless `ref` is too close to parallel to that (within ~8deg,
// where the cross product would be too small to normalize reliably), in
// which case falls back to (1,0,0) -- `ref` can't be close to parallel to
// BOTH, since they're themselves perpendicular.
template <typename ValueType>
void orthonormal_basis_perpendicular_to(
    ValueType ref_x, ValueType ref_y, ValueType ref_z,
    ValueType& out_ux, ValueType& out_uy, ValueType& out_uz,
    ValueType& out_vx, ValueType& out_vy, ValueType& out_vz) noexcept {
    ValueType seed_x = ValueType{0}, seed_y = ValueType{0}, seed_z = ValueType{1};
    // |ref_z| close to 1 means ref is nearly parallel to the (0,0,1) seed.
    if (ref_z > static_cast<ValueType>(0.99) || ref_z < static_cast<ValueType>(-0.99)) {
        seed_x = ValueType{1};
        seed_y = ValueType{0};
        seed_z = ValueType{0};
    }

    // out_u = normalize(ref x seed)
    ValueType ux = ref_y * seed_z - ref_z * seed_y;
    ValueType uy = ref_z * seed_x - ref_x * seed_z;
    ValueType uz = ref_x * seed_y - ref_y * seed_x;
    const ValueType u_mag = std::sqrt(ux * ux + uy * uy + uz * uz);
    ux /= u_mag; uy /= u_mag; uz /= u_mag;

    // out_v = ref x out_u -- already unit length since ref and out_u are
    // unit and perpendicular.
    const ValueType vx = ref_y * uz - ref_z * uy;
    const ValueType vy = ref_z * ux - ref_x * uz;
    const ValueType vz = ref_x * uy - ref_y * ux;

    out_ux = ux; out_uy = uy; out_uz = uz;
    out_vx = vx; out_vy = vy; out_vz = vz;
}

} // namespace edgeneuro

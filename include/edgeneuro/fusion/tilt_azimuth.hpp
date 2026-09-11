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
// history / SESSION_LOG.md 2026-09-04 for the real hardware session (a forward-
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

// tilt_azimuth()'s azimuth assumes whatever two directions the caller cares
// about are perpendicular (basis_u/basis_v built via
// orthonormal_basis_perpendicular_to). Real captured shoulder data
// (SESSION_LOG.md, 2026-09-04 "FORWARD_RAISE/ABDUCTION_LEFT cross-talk" finding)
// shows that assumption doesn't hold: two independent 5-repeat capture
// sessions both measured only ~29deg of azimuth separation between a real
// FORWARD_RAISE and a real ABDUCTION_LEFT reading, not 90deg -- and the
// measurement noise floor across those repeats was only ~3-6deg, an order
// of magnitude too small to explain a 60deg gap from the idealized value.
// Most likely cause: large-angle 3D rotation composition doesn't preserve
// the local (small-angle) orthogonality between two rotation axes (see
// this file's header comment on the ISB/Tilt-and-Torsion literature),
// possibly compounded by scapulohumeral rhythm neither of this project's 2
// IMUs can isolate from pure glenohumeral rotation.
//
// ObliqueBasis/oblique_decompose sidestep that assumption entirely: instead
// of projecting onto an idealized perpendicular pair, they solve against
// the two REAL calibration directions actually measured, whatever the true
// angle between them is. fwd/abd are Gram-matrix (least-squares, exact
// here since the input's tangent-plane projection lies exactly in
// span{pf, pa}) coordinates in that oblique basis: 1.0 means "exactly as
// far in that direction as its calibration reading", 0.0 means "at ref".
template <typename ValueType>
struct ObliqueBasis {
    ValueType ref_x, ref_y, ref_z;
    ValueType pf_x, pf_y, pf_z; // fwd calibration reading, projected onto the tangent plane at ref
    ValueType pa_x, pa_y, pa_z; // abd calibration reading, projected onto the tangent plane at ref
    ValueType a11, a12, a22;    // Gram matrix of {pf, pa}: a11=pf.pf, a12=pf.pa, a22=pa.pa
    ValueType inv_det;          // 1/(a11*a22 - a12*a12), precomputed once since this basis is reused every sample
    ValueType tilt_fwd, tilt_abd; // each calibration pose's own tilt (radians) from ref -- lets a caller
                                  // scale fwd/abd (dimensionless: 1.0 = "at that calibration pose") back
                                  // into a real angle-equivalent, e.g. fwd*tilt_fwd
};

// ref/fwd/abd must all be unit vectors (pre-normalized by the caller, same
// convention as tilt_azimuth above) and fwd/abd must each differ from ref
// and from each other (any two distinct real calibration poses satisfy
// this in practice -- degenerate only if fwd and abd project to the exact
// same or exactly opposite tangent direction, which a real capture won't).
template <typename ValueType>
ObliqueBasis<ValueType> make_oblique_basis(
    ValueType ref_x, ValueType ref_y, ValueType ref_z,
    ValueType fwd_x, ValueType fwd_y, ValueType fwd_z,
    ValueType abd_x, ValueType abd_y, ValueType abd_z) noexcept {
    ObliqueBasis<ValueType> basis{};
    basis.ref_x = ref_x; basis.ref_y = ref_y; basis.ref_z = ref_z;

    const ValueType fwd_dot_ref = fwd_x * ref_x + fwd_y * ref_y + fwd_z * ref_z;
    basis.pf_x = fwd_x - fwd_dot_ref * ref_x;
    basis.pf_y = fwd_y - fwd_dot_ref * ref_y;
    basis.pf_z = fwd_z - fwd_dot_ref * ref_z;

    const ValueType abd_dot_ref = abd_x * ref_x + abd_y * ref_y + abd_z * ref_z;
    basis.pa_x = abd_x - abd_dot_ref * ref_x;
    basis.pa_y = abd_y - abd_dot_ref * ref_y;
    basis.pa_z = abd_z - abd_dot_ref * ref_z;

    basis.a11 = basis.pf_x * basis.pf_x + basis.pf_y * basis.pf_y + basis.pf_z * basis.pf_z;
    basis.a12 = basis.pf_x * basis.pa_x + basis.pf_y * basis.pa_y + basis.pf_z * basis.pa_z;
    basis.a22 = basis.pa_x * basis.pa_x + basis.pa_y * basis.pa_y + basis.pa_z * basis.pa_z;
    const ValueType det = basis.a11 * basis.a22 - basis.a12 * basis.a12;
    basis.inv_det = ValueType{1} / det;

    ValueType fwd_dot_ref_clamped = fwd_dot_ref;
    if (fwd_dot_ref_clamped > ValueType{1}) fwd_dot_ref_clamped = ValueType{1};
    if (fwd_dot_ref_clamped < ValueType{-1}) fwd_dot_ref_clamped = ValueType{-1};
    basis.tilt_fwd = std::acos(fwd_dot_ref_clamped);
    ValueType abd_dot_ref_clamped = abd_dot_ref;
    if (abd_dot_ref_clamped > ValueType{1}) abd_dot_ref_clamped = ValueType{1};
    if (abd_dot_ref_clamped < ValueType{-1}) abd_dot_ref_clamped = ValueType{-1};
    basis.tilt_abd = std::acos(abd_dot_ref_clamped);
    return basis;
}

template <typename ValueType>
struct ObliqueCoeffs {
    ValueType fwd;
    ValueType abd;
};

// accel_x/y/z must be a unit vector (pre-normalized by the caller).
template <typename ValueType>
ObliqueCoeffs<ValueType> oblique_decompose(
    const ObliqueBasis<ValueType>& basis,
    ValueType accel_x, ValueType accel_y, ValueType accel_z) noexcept {
    const ValueType dot_ref = accel_x * basis.ref_x + accel_y * basis.ref_y + accel_z * basis.ref_z;
    const ValueType px = accel_x - dot_ref * basis.ref_x;
    const ValueType py = accel_y - dot_ref * basis.ref_y;
    const ValueType pz = accel_z - dot_ref * basis.ref_z;

    const ValueType b1 = px * basis.pf_x + py * basis.pf_y + pz * basis.pf_z;
    const ValueType b2 = px * basis.pa_x + py * basis.pa_y + pz * basis.pa_z;

    const ValueType fwd = (b1 * basis.a22 - b2 * basis.a12) * basis.inv_det;
    const ValueType abd = (b2 * basis.a11 - b1 * basis.a12) * basis.inv_det;
    return ObliqueCoeffs<ValueType>{fwd, abd};
}

// oblique_decompose()'s fwd/abd are exact AT the two calibration poses
// (scaling each by its own tilt_fwd/tilt_abd correctly recovers that pose's
// real angle -- see make_oblique_basis), but for a direction OFF either
// calibration axis, that same "coefficient times the calibration's own
// (generally large) tilt" scaling can overshoot the real angle by a
// wide margin: found on real captured data (SESSION_LOG.md, 2026-09-04) where a
// real ~16deg shoulder drift during ELBOW_FLEXION (a pose that plays no
// part in building the basis) scaled out to a ~35deg pitch_equiv --
// more than double the real tilt -- because that drift's direction, while
// small in magnitude, happens to lie partway along the (large-angle)
// FORWARD_RAISE calibration direction.
//
// oblique_decompose_scaled fixes this by keeping the OBLIQUE decomposition
// for DIRECTION only, and substituting the real, independently-measured
// tilt (acos-based, same as tilt_azimuth()'s tilt -- physically exact,
// never amplified) for MAGNITUDE: normalize (fwd, abd) to a unit direction,
// then scale by the real tilt. Exact at both calibration poses (fwd/abd
// there are already (1,0)/(0,1), unit vectors, so this is a no-op beyond
// confirming tilt==tilt_fwd/tilt_abd), and bounded by the real tilt
// everywhere else -- it can no longer overshoot the way the raw
// coefficient scaling did.
//
// Unlike oblique_decompose/tilt_azimuth above, accel_x/y/z here need NOT be
// pre-normalized -- normalized internally, since tilt = acos(dot_ref) is
// only correct for a unit vector and a real accelerometer reading is
// rarely exactly unit magnitude (this project's real captures run
// ~0.98-1.03g). Getting this wrong is exactly how this function's own
// motivating bug was almost reintroduced while writing its test: an
// unnormalized real reading (|v|=1.029) silently produced a scaled-down
// coefficient (both fwd and abd off by the same ~1/|v| factor) that looked
// plausible but was measurably wrong against the independently-computed
// expected value.
template <typename ValueType>
ObliqueCoeffs<ValueType> oblique_decompose_scaled(
    const ObliqueBasis<ValueType>& basis,
    ValueType accel_x, ValueType accel_y, ValueType accel_z) noexcept {
    const ValueType mag_in = std::sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    const ValueType nx = accel_x / mag_in;
    const ValueType ny = accel_y / mag_in;
    const ValueType nz = accel_z / mag_in;

    ValueType dot_ref = nx * basis.ref_x + ny * basis.ref_y + nz * basis.ref_z;
    if (dot_ref > ValueType{1}) dot_ref = ValueType{1};
    if (dot_ref < ValueType{-1}) dot_ref = ValueType{-1};
    const ValueType tilt = std::acos(dot_ref);

    const auto raw = oblique_decompose(basis, nx, ny, nz);
    const ValueType mag = std::sqrt(raw.fwd * raw.fwd + raw.abd * raw.abd);
    // Direction is undefined exactly at tilt=0 (same convention as
    // tilt_azimuth()'s azimuth -- returned as 0 rather than a 0/0 NaN).
    if (mag < static_cast<ValueType>(1e-6)) {
        return ObliqueCoeffs<ValueType>{ValueType{0}, ValueType{0}};
    }
    return ObliqueCoeffs<ValueType>{tilt * raw.fwd / mag, tilt * raw.abd / mag};
}

} // namespace edgeneuro

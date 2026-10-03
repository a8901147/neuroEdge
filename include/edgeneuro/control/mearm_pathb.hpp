#pragma once

#include <array>
#include <cmath>

#include "edgeneuro/control/mearm_linkage.hpp"

namespace edgeneuro::mearm::pathb {

// C++ port of tools/mujoco_bridge/mearm_pathb.py's ctrl_from_sensors: the
// upper-arm accelerometer's gravity vector plus the firmware's elbow reading ->
// MeArm (base, shoulder, elbow) ctrl, using the SAME validated math as the
// Python/MuJoCo path: spherical (tilt, azimuth) decode about the calibrated
// HANG axis, pole and behind-body fades, and the shoulder/elbow linkage
// projection. Python is the source of truth; tests/test_mearm_pathb.cpp checks
// this against data/pathb_golden.csv (the 2026-09-24 real log through Python).
//
// Outputs are MODEL ctrl radians (mearm_scene.xml actuator space), NOT servo
// pulse widths: a servo-side mapping from these to real pulses (polarity and
// limits measured on the real arm, SESSION_LOG TODO C) is a separate, later step.
// Not wired into phase3_control_loop yet.
constexpr float kBaseLimit = 1.08210414f;
constexpr float kBaseSwing = 0.9f;                  // model base rotation for the calibrated LEFT/RIGHT
constexpr float kElbowSwingRad = 2.25147473507f;    // radians(129deg), 9/05 real capture (relative swing only)
constexpr float kPi = 3.14159265358979f;
constexpr float kTiltFadeInLoDeg = 8.0f, kTiltFadeInHiDeg = 20.0f;
constexpr float kBehindFadeLoDeg = 90.0f, kBehindFadeHiDeg = 135.0f;
constexpr float kMinForwardTiltDeg = 20.0f;

using Vec3 = std::array<float, 3>;

namespace detail {

inline float deg2rad(float d) { return d * kPi / 180.0f; }
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline Vec3 mul(const Vec3& a, float s) { return {a[0] * s, a[1] * s, a[2] * s}; }
inline bool finite3(const Vec3& v) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]); }

// false for a zero-length or non-finite vector
inline bool unit(const Vec3& v, Vec3& out) {
    if (!finite3(v)) return false;
    const float n = std::sqrt(dot(v, v));
    if (!(n >= 1e-9f)) return false;
    out = mul(v, 1.0f / n);
    return true;
}

inline float smoothstep(float x) {
    x = clampf(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

// Piecewise-linear through N (x, y) anchors sorted by ascending x, extrapolating
// along the end segments, then clamped to [lo, hi] (same as interp_anchors).
template <std::size_t N>
inline float interp(const float (&xs)[N], const float (&ys)[N], float x, float lo, float hi) {
    std::size_t i = 0;
    if (x <= xs[0]) {
        i = 0;
    } else if (x >= xs[N - 1]) {
        i = N - 2;
    } else {
        while (i + 2 < N && x > xs[i + 1]) ++i;
    }
    const float y = ys[i] + (x - xs[i]) * (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]);
    return clampf(y, lo < hi ? lo : hi, lo < hi ? hi : lo);
}

}  // namespace detail

struct Ctrl {
    float base;
    float shoulder;
    float elbow;
};

// Everything derived from the saved calibration (HANG, FORWARD, LEFT, RIGHT raw
// gravity vectors and the straight-arm elbow reading), built once.
struct Calibration {
    Vec3 h{}, e1{}, e2{};
    float tilt_forward = 0.0f, az_left = 0.0f, az_right = 0.0f, zero_elbow = 0.0f;

    // false (and `out` untouched) if the calibration is unusable: a zero/non-finite
    // vector, FORWARD < 20deg from HANG, or LEFT/RIGHT not on opposite sides of FORWARD.
    static bool make(const Vec3& hang, const Vec3& forward, const Vec3& left, const Vec3& right,
                     float zero_elbow, Calibration& out) {
        using namespace detail;
        if (!std::isfinite(zero_elbow)) return false;
        Calibration c;
        Vec3 fwd_u, left_u;
        if (!unit(hang, c.h) || !unit(forward, fwd_u) || !unit(left, left_u)) return false;
        Vec3 right_u;
        if (!unit(right, right_u)) return false;
        if (!unit(c.tangent(fwd_u), c.e1)) return false;
        const Vec3 t_left = c.tangent(left_u);
        if (!unit(sub(t_left, mul(c.e1, dot(t_left, c.e1))), c.e2)) return false;
        c.tilt_forward = std::acos(clampf(dot(c.h, fwd_u), -1.0f, 1.0f));
        float tilt_unused;
        c.decode(left, tilt_unused, c.az_left);
        c.decode(right, tilt_unused, c.az_right);
        if (c.tilt_forward < deg2rad(kMinForwardTiltDeg)) return false;
        if (!(c.az_left > 0.1f && c.az_right < -0.1f)) return false;
        c.zero_elbow = zero_elbow;
        out = c;
        return true;
    }

    Vec3 tangent(const Vec3& v) const { return detail::sub(v, detail::mul(h, detail::dot(v, h))); }

    // (tilt, azimuth) radians: tilt = angle from HANG (>= 0), azimuth = direction of
    // the tilt, 0 = FORWARD, + = LEFT. Returns false for an unusable raw vector.
    bool decode(const Vec3& raw, float& tilt, float& az) const {
        using namespace detail;
        Vec3 v;
        if (!unit(raw, v)) return false;
        tilt = std::acos(clampf(dot(v, h), -1.0f, 1.0f));
        const Vec3 t = tangent(v);
        if (std::sqrt(dot(t, t)) < 1e-9f) {
            az = 0.0f;
        } else {
            az = std::atan2(dot(t, e2), dot(t, e1));
        }
        return true;
    }
};

// The model elbow command the person's bend asks for, BEFORE the MuJoCo model's linkage projection (zero_elbow ->
// extended, zero_elbow + the measured 129 deg swing -> folded). A garbage reading means straight.
inline float elbow_request(const Calibration& cal, float elbow_bend) {
    using namespace detail;
    if (!std::isfinite(elbow_bend)) return linkage::kElbowExtended;
    const float f = clampf((elbow_bend - cal.zero_elbow) / kElbowSwingRad, 0.0f, 1.0f);
    return linkage::kElbowExtended + f * (linkage::kElbowFolded - linkage::kElbowExtended);
}

// (pole, front), each 0..1: the base fades in as the arm leaves HANG (azimuth is meaningless near hanging) and the whole
// arm fades back to rest behind the body. Shared by ctrl_from_sensors and real::base_pulse (same as mearm_pathb.fades).
struct Fades {
    float pole, front;
};
inline Fades fades(float tilt, float az) {
    using namespace detail;
    const float front = clampf((deg2rad(kBehindFadeHiDeg) - std::fabs(az)) /
                                   (deg2rad(kBehindFadeHiDeg) - deg2rad(kBehindFadeLoDeg)),
                               0.0f, 1.0f);
    const float pole = smoothstep((tilt - deg2rad(kTiltFadeInLoDeg)) /
                                  (deg2rad(kTiltFadeInHiDeg) - deg2rad(kTiltFadeInLoDeg)));
    return {pole, front};
}

inline Ctrl ctrl_from_sensors(const Calibration& cal, const Vec3& upper_raw, float elbow_bend) {
    using namespace detail;
    float base = 0.0f, shoulder = linkage::kShoulderRest;   // no valid reading -> rest, never a guess
    float tilt = 0.0f, az = 0.0f;
    if (finite3(upper_raw) && std::sqrt(dot(upper_raw, upper_raw)) >= 1e-6f &&
        cal.decode(upper_raw, tilt, az)) {
        const Fades f = fades(tilt, az);
        const float pole = f.pole, front = f.front;
        const float bx[3] = {cal.az_right, 0.0f, cal.az_left};
        const float by[3] = {-kBaseSwing, 0.0f, kBaseSwing};
        base = interp(bx, by, az, -kBaseLimit, kBaseLimit) * pole * front;
        const float sx[2] = {0.0f, cal.tilt_forward};
        const float sy[2] = {linkage::kShoulderRest, linkage::kShoulderRaised};
        shoulder = interp(sx, sy, tilt * front, linkage::kShoulderRaised, linkage::kShoulderRest);
    }
    return {base, shoulder, linkage::project_elbow(shoulder, elbow_request(cal, elbow_bend))};
}

}  // namespace edgeneuro::mearm::pathb

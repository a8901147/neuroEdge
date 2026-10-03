#pragma once

#include <cmath>
#include <limits>

#include "edgeneuro/control/mearm_angle_data.hpp"
#include "edgeneuro/control/mearm_envelope.hpp"
#include "edgeneuro/control/mearm_pathb.hpp"

namespace edgeneuro::mearm::real {

// C++ port of tools/mujoco_bridge/mearm_real.py in STRETCH mode (the user's choice, SESSION_LOG 2026-09-27): upper-arm
// gravity vector + elbow reading -> REAL (shoulder, elbow) servo pulses, always inside the measured safe envelope.
//   Path B decode -> model shoulder command (spread over the arm's safe shoulder range: "stretch") + the elbow the person
//   asks for (the model's RELATIVE elbow = the angle between upper arm and forearm; NOT the MuJoCo linkage projection)
//   -> measured link-angle lines (mearm_angle_data.hpp) -> pulses -> PulseEnvelope.
// The base is base_pulse() below; the claw is not here (grip -> claw_map).

constexpr float kPi = 3.14159265358979f;

// Model command (radians) -> (shoulder, elbow) pulse widths, not yet clamped. The real arm's links are driven through
// parallel linkages: upper-arm elevation = 90deg - shoulder, forearm elevation = 90deg - (shoulder + elbow).
struct RawPulses {
    float shoulder;
    float elbow;
};

inline RawPulses pulses_from_model_ctrl(float shoulder_ctrl, float elbow_ctrl) {
    using namespace angle_data;
    const float upper = 90.0f - shoulder_ctrl * 180.0f / kPi;
    const float forearm = 90.0f - (shoulder_ctrl + elbow_ctrl) * 180.0f / kPi;
    return {1500.0f + (upper - kUpperArmAt1500Deg) / kUpperArmSlopeDegPerUs,
            1500.0f + (forearm - kForearmAt1500Deg) / kForearmSlopeDegPerUs};
}

// The model shoulder command a shoulder pulse corresponds to (inverse of the upper-arm line).
inline float shoulder_ctrl_at_pulse(float pulse_us) {
    using namespace angle_data;
    const float upper = kUpperArmAt1500Deg + kUpperArmSlopeDegPerUs * (pulse_us - 1500.0f);
    return (90.0f - upper) * kPi / 180.0f;
}

// The elbow window used to SPREAD the person's raise, continuous in the shoulder pulse and never outside the measured
// envelope (2026-10-03: the envelope's own window steps at its measured shoulder positions, so spreading across it made
// one degree of elbow bend jump the elbow servo up to 350 us). At each measured shoulder: the lowest top / highest
// bottom of that point and its neighbours, joined by straight lines. Same as mearm_real.py's smooth_elbow_window.
struct ElbowWindow {
    float top, bottom;
};
inline ElbowWindow smooth_elbow_window(float shoulder_us) {
    namespace E = envelope_data;
    auto top_at = [](unsigned i) {
        unsigned v = E::kElbowHiUs[i];
        if (i > 0u && E::kElbowHiUs[i - 1u] < v) v = E::kElbowHiUs[i - 1u];
        if (i + 1u < E::kCount && E::kElbowHiUs[i + 1u] < v) v = E::kElbowHiUs[i + 1u];
        return static_cast<float>(v);
    };
    auto bottom_at = [](unsigned i) {
        unsigned v = E::kElbowLoUs[i];
        if (i > 0u && E::kElbowLoUs[i - 1u] > v) v = E::kElbowLoUs[i - 1u];
        if (i + 1u < E::kCount && E::kElbowLoUs[i + 1u] > v) v = E::kElbowLoUs[i + 1u];
        return static_cast<float>(v);
    };
    const float lo_s = static_cast<float>(E::kShoulderUs[0]), hi_s = static_cast<float>(E::kShoulderUs[E::kCount - 1u]);
    const float s = shoulder_us < lo_s ? lo_s : (shoulder_us > hi_s ? hi_s : shoulder_us);
    for (unsigned i = 0; i + 1u < E::kCount; ++i) {
        const float a = static_cast<float>(E::kShoulderUs[i]), b = static_cast<float>(E::kShoulderUs[i + 1u]);
        if (a <= s && s <= b) {
            const float f = (s - a) / (b - a);
            return {top_at(i) + f * (top_at(i + 1u) - top_at(i)), bottom_at(i) + f * (bottom_at(i + 1u) - bottom_at(i))};
        }
    }
    return {top_at(E::kCount - 1u), bottom_at(E::kCount - 1u)};
}

// height_reach (2026-10-03, the user's design, checked on the real arm; mearm_real.py's default mode): the MEArm's forearm
// servo sets the claw's HEIGHT and its upper-arm servo its REACH, so the person's arm drives them crosswise --
//   raising the arm (Path B's tilt, hanging -> raised, with its fades) lowers the ELBOW servo across the elbow window the
//   envelope allows at the current shoulder pulse (claw up);
//   bending the elbow (straight -> fully bent) raises the SHOULDER servo across the measured shoulder range (reach).
// Hanging + straight = (1500, 1500) = the R start pose. An invalid upper-arm reading -> the rest pose, whatever the elbow.
inline Pulses pulses(const pathb::Calibration& cal, const pathb::Vec3& upper_raw, float elbow_bend) {
    const auto& env = envelope();
    const bool valid = std::isfinite(upper_raw[0]) && std::isfinite(upper_raw[1]) && std::isfinite(upper_raw[2]) &&
                       std::sqrt(upper_raw[0] * upper_raw[0] + upper_raw[1] * upper_raw[1] + upper_raw[2] * upper_raw[2]) >= 1e-6f;
    if (!valid) return env.clamp(static_cast<float>(kShoulderRestUs), static_cast<float>(kElbowRestUs));
    const auto ctrl = pathb::ctrl_from_sensors(cal, upper_raw, elbow_bend);
    float f_raise = (linkage::kShoulderRest - ctrl.shoulder) / (linkage::kShoulderRest - linkage::kShoulderRaised);
    f_raise = f_raise < 0.0f ? 0.0f : (f_raise > 1.0f ? 1.0f : f_raise);
    float f_bend = 0.0f;
    if (std::isfinite(elbow_bend)) {
        f_bend = (pathb::elbow_request(cal, elbow_bend) - linkage::kElbowExtended) /
                 (linkage::kElbowFolded - linkage::kElbowExtended);
        f_bend = f_bend < 0.0f ? 0.0f : (f_bend > 1.0f ? 1.0f : f_bend);
    }
    const float s_lo = static_cast<float>(env.shoulder_min()), s_hi = static_cast<float>(env.shoulder_max());
    const unsigned s = env.clamp(s_lo + f_bend * (s_hi - s_lo), static_cast<float>(kElbowRestUs)).shoulder;
    const float sf = static_cast<float>(s);
    const ElbowWindow w = smooth_elbow_window(sf);
    return env.clamp(sf, w.top - f_raise * (w.top - w.bottom));
}

// Base, measured on the real arm 2026-09-28: usable 500..2500 us, 1700 us turns LEFT (seen from behind the arm).
constexpr unsigned kRestBaseUs = 1500u;
constexpr unsigned kBaseMinUs = 500u, kBaseMaxUs = 2500u;

// Where the base hits its ends: the arm's azimuth (rad, + = left) at its comfortable left / right reach (2026-10-03, the
// user's choice -- the base keeps its whole range, a turn of the arm moves it less). Same as mearm_real.py.
struct BaseReach {
    float left, right;
};

// Without a measured reach: exactly Path B's own base mapping (calibrated LEFT/RIGHT -> +-kBaseSwing, clamped at
// +-kBaseLimit), i.e. ~+-34 deg for the 9/13 calibration.
inline BaseReach default_base_reach(const pathb::Calibration& cal) {
    return {cal.az_left * pathb::kBaseLimit / pathb::kBaseSwing, cal.az_right * pathb::kBaseLimit / pathb::kBaseSwing};
}

// From the measured reach poses' raw vectors; false if they are not on both sides of FORWARD.
inline bool make_base_reach(const pathb::Calibration& cal, const pathb::Vec3& left_raw, const pathb::Vec3& right_raw,
                            BaseReach& out) {
    float t = 0.0f, l = 0.0f, r = 0.0f;
    if (!cal.decode(left_raw, t, l) || !cal.decode(right_raw, t, r)) return false;
    if (!(l > 0.1f && r < -0.1f)) return false;
    out = {l, r};
    return true;
}

inline unsigned base_pulse(const pathb::Calibration& cal, const pathb::Vec3& upper_raw, const BaseReach& reach) {
    float tilt = 0.0f, az = 0.0f;
    const bool finite = std::isfinite(upper_raw[0]) && std::isfinite(upper_raw[1]) && std::isfinite(upper_raw[2]);
    if (!finite || std::sqrt(upper_raw[0] * upper_raw[0] + upper_raw[1] * upper_raw[1] + upper_raw[2] * upper_raw[2]) < 1e-6f ||
        !cal.decode(upper_raw, tilt, az)) {
        return kRestBaseUs;                                       // no valid reading -> rest, never a guess
    }
    const pathb::Fades f = pathb::fades(tilt, az);
    const float xs[3] = {reach.right, 0.0f, reach.left};
    const float ys[3] = {-1.0f, 0.0f, 1.0f};
    const float frac = pathb::detail::interp(xs, ys, az, -1.0f, 1.0f) * f.pole * f.front;
    const float half = 0.5f * static_cast<float>(kBaseMaxUs - kBaseMinUs);
    float us = static_cast<float>(kRestBaseUs) + frac * half;
    us = us < static_cast<float>(kBaseMinUs) ? static_cast<float>(kBaseMinUs)
                                             : (us > static_cast<float>(kBaseMaxUs) ? static_cast<float>(kBaseMaxUs) : us);
    return static_cast<unsigned>(std::floor(us + 0.5f));
}

inline unsigned base_pulse(const pathb::Calibration& cal, const pathb::Vec3& upper_raw) {
    return base_pulse(cal, upper_raw, default_base_reach(cal));
}

}  // namespace edgeneuro::mearm::real

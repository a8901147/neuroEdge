#pragma once

#include <cmath>

namespace edgeneuro::mearm::linkage {

// C++ port of tools/mujoco_bridge/mearm_pathb.py's shoulder/elbow linkage
// coupling. A MeArm's claw-levelling parallel linkage makes shoulder and elbow
// NOT independent: only a diagonal band of (shoulder + elbow) is reachable.
// Python is the source of truth; tests/test_mearm_linkage.cpp checks this port
// against data/linkage_golden.csv (generated from the Python).
//
// NOT wired into phase3_control_loop yet: the band limits below are MeArmPilot's
// (measured on THEIR unit), not this arm's -- SESSION_LOG TODO C.
constexpr float kShoulderRest = 0.898057932f;   // model shoulder ctrl, arm hanging
constexpr float kShoulderRaised = -0.141261412f;
constexpr float kElbowExtended = 0.994603031f;
constexpr float kElbowFolded = 2.61715444f;
constexpr float kToolLockSum = 1.57079632679f;  // tool = pi/2 - shoulder - elbow
constexpr float kToolLimitLo = -0.940003288f;
constexpr float kToolLimitHi = -0.286958051f;
constexpr float kMargin = 0.05f;                // keep off the exact limit

constexpr float band_lo() { return kToolLockSum - kToolLimitHi + kMargin; }
constexpr float band_hi() { return kToolLockSum - kToolLimitLo - kMargin; }

namespace detail {
// The shoulder actually commanded is always inside its actuator range (Path B
// clamps it); clamp here too so a bad value can't make the window empty or
// non-finite. NaN -> rest pose (neutral).
inline float sane_shoulder(float s) {
    if (std::isnan(s)) return kShoulderRest;
    if (s < kShoulderRaised) return kShoulderRaised;
    if (s > kShoulderRest) return kShoulderRest;
    return s;
}
}  // namespace detail

inline float window_lo(float shoulder) {
    const float v = band_lo() - detail::sane_shoulder(shoulder);
    return v > kElbowExtended ? v : kElbowExtended;
}
inline float window_hi(float shoulder) {
    const float v = band_hi() - detail::sane_shoulder(shoulder);
    return v < kElbowFolded ? v : kElbowFolded;
}

// Fits the requested elbow into the window the linkage allows at this
// shoulder, PROPORTIONALLY (not a hard clamp: a hard clamp saturates half-way
// through a flexion at low shoulder heights).
inline float project_elbow(float shoulder, float request) {
    const float lo = window_lo(shoulder);
    const float hi = window_hi(shoulder);
    float fraction = 0.0f;  // NaN request -> fully extended
    if (!std::isnan(request)) {
        fraction = (request - kElbowExtended) / (kElbowFolded - kElbowExtended);
        if (fraction < 0.0f) fraction = 0.0f;
        else if (fraction > 1.0f) fraction = 1.0f;
    }
    return lo + fraction * (hi - lo);
}

}  // namespace edgeneuro::mearm::linkage

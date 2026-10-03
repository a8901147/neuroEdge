#pragma once

#include "edgeneuro/control/servo_angle_map.hpp"
#include "edgeneuro/control/servo_startup_ramp.hpp"

namespace edgeneuro::mearm {

// The four real-MEArm servo maps, extracted from phase3_control_loop_main.cpp
// so host tests can check their safety properties (tests/test_mearm_servo_maps.cpp).
// Pulse ranges = measured on the assembled arm with servo_limit_finder_4ch
// (SESSION_LOG 2026-09-21/22), each measured with the OTHER servos at centre.
// The shoulder x elbow COMBINATION is NOT measured yet (SESSION_LOG TODO C):
// the two are coupled by the claw-levelling linkage, so do not drive both
// independently on the real arm until that is done.
//
// Polarity: shoulder inferred, base/elbow/claw unverified live.
// Value ranges are the human-side sensor ranges (run_demo_live.py's).
constexpr unsigned kBaseLoUs = 500u, kBaseHiUs = 2500u;
constexpr unsigned kShoulderLoUs = 1200u, kShoulderHiUs = 2100u;
constexpr unsigned kElbowLoUs = 500u, kElbowHiUs = 1850u;
// claw: measured travel 1300 (open) .. 1600 (closed), 2026-09-23; limited to 1300..1500 by the user (2026-10-03) --
// a full grip commands 1500, and nothing ever sends more
constexpr unsigned kClawLoUs = 1300u, kClawHiUs = 1500u;

inline const ServoAngleMap& base_map() {      // shoulder_roll -> base
    static constexpr ServoAngleMap m(-0.8727f, 2.2515f, kBaseLoUs, kBaseHiUs);
    return m;
}
inline const ServoAngleMap& shoulder_map() {  // shoulder_pitch -> shoulder
    static constexpr ServoAngleMap m(-3.0892f, 1.0472f, kShoulderLoUs, kShoulderHiUs);
    return m;
}
inline const ServoAngleMap& elbow_map() {     // elbow -> elbow
    static constexpr ServoAngleMap m(-1.0472f, 1.28f, kElbowLoUs, kElbowHiUs);
    return m;
}
inline const ServoAngleMap& claw_map() {      // grip (0..1) -> claw
    static constexpr ServoAngleMap m(0.0f, 1.0f, kClawLoUs, kClawHiUs);
    return m;
}

// Start-up: every servo starts at its own rest pulse and walks to its target slowly
// (servo_startup_ramp.hpp says what that can and cannot do).
// Rest pulses were chosen 2026-09-26 by looking at the real arm with base/shoulder/elbow
// at 1500us (a mid-reach pose, nothing at a mechanical limit) and the claw open at 1300us.
// They are a reasonable start, NOT a measured "safe" pose. The claw's rest equals what a
// relaxed hand (grip = 0) already commands, so it does not move at boot. 1300 is the claw's
// measured END of travel: holding a servo against its stop can buzz and heat it -- watch for
// that, and pull the rest in by a few tens of us if it does.
// The two rates are still first guesses; tune them on the arm (SESSION_LOG).
constexpr unsigned kBaseRestUs = 1500u;
constexpr unsigned kShoulderRestUs = 1500u;
constexpr unsigned kElbowRestUs = 1500u;
constexpr unsigned kClawRestUs = 1300u;
constexpr float kStartRateUsPerS = 300.0f;    // slow walk from rest to the first target
constexpr float kTrackRateUsPerS = 6000.0f;   // afterwards: about the SG92R's own speed (0.1 s / 60 deg)
constexpr float kArrivedWithinUs = 2.0f;

inline ServoStartupRamp base_ramp() {
    return ServoStartupRamp(kBaseRestUs, kBaseLoUs, kBaseHiUs, kStartRateUsPerS, kTrackRateUsPerS, kArrivedWithinUs);
}
inline ServoStartupRamp shoulder_ramp() {
    return ServoStartupRamp(kShoulderRestUs, kShoulderLoUs, kShoulderHiUs, kStartRateUsPerS, kTrackRateUsPerS, kArrivedWithinUs);
}
inline ServoStartupRamp elbow_ramp() {
    return ServoStartupRamp(kElbowRestUs, kElbowLoUs, kElbowHiUs, kStartRateUsPerS, kTrackRateUsPerS, kArrivedWithinUs);
}
inline ServoStartupRamp claw_ramp() {
    return ServoStartupRamp(kClawRestUs, kClawLoUs, kClawHiUs, kStartRateUsPerS, kTrackRateUsPerS, kArrivedWithinUs);
}

}  // namespace edgeneuro::mearm

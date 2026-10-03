#pragma once

#include "edgeneuro/control/mearm_envelope.hpp"
#include "edgeneuro/control/servo_startup_ramp.hpp"

namespace edgeneuro::mearm {

// One servo tick of the shoulder and elbow TOGETHER, so the real arm stays inside the measured safe envelope on every
// tick, not only at its target. (2026-09-28: with the envelope up to shoulder 2100, where the elbow may only go up to
// 1000, two independent ramps could briefly pass through e.g. (2025, 1300) on a fast raise -- the elbow's linkage hits
// the upper arm there.)
//
// Each ramp proposes its next step; the first of these that is inside the envelope is taken:
//   both move  ->  the elbow moves, the shoulder waits  ->  the shoulder moves, the elbow waits  ->  both wait.
// A ramp whose step is not taken is left exactly where it was (no jump later). Starting from a pose inside the envelope
// (rest, 1500/1500, is) every tick stays inside; targets are drive::command's, already clamped into the envelope.
inline Pulses joint_step(ServoStartupRamp& shoulder, ServoStartupRamp& elbow, float shoulder_target_us,
                         float elbow_target_us, float dt_s) noexcept {
    const auto& env = envelope();
    const auto inside = [&env](unsigned s, unsigned e) {
        const Pulses p = env.clamp(static_cast<float>(s), static_cast<float>(e));
        return p.shoulder == s && p.elbow == e;
    };
    ServoStartupRamp s_next = shoulder, e_next = elbow;
    const unsigned s_now = shoulder.current_us(), e_now = elbow.current_us();
    const unsigned s_new = s_next.step(shoulder_target_us, dt_s);
    const unsigned e_new = e_next.step(elbow_target_us, dt_s);
    if (inside(s_new, e_new)) {
        shoulder = s_next;
        elbow = e_next;
        return {s_new, e_new};
    }
    if (inside(s_now, e_new)) {
        elbow = e_next;
        return {s_now, e_new};
    }
    if (inside(s_new, e_now)) {
        shoulder = s_next;
        return {s_new, e_now};
    }
    return {s_now, e_now};
}

}  // namespace edgeneuro::mearm

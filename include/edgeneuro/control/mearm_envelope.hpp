#pragma once

#include "edgeneuro/control/mearm_envelope_data.hpp"
#include "edgeneuro/control/mearm_servo_maps.hpp"
#include "edgeneuro/control/pulse_envelope.hpp"

namespace edgeneuro::mearm {

// The real MEArm's measured safe envelope (see mearm_envelope_data.hpp -- generated from the raw measurement files).
// Used by real::pulses (targets) and joint_step (every servo tick) in phase3_control_loop with EDGENEURO_DRIVE_SERVOS.
inline const PulseEnvelope& envelope() {
    static constexpr PulseEnvelope e(envelope_data::kShoulderUs, envelope_data::kElbowLoUs, envelope_data::kElbowHiUs,
                                 envelope_data::kCount, kShoulderRestUs, kElbowRestUs);
    return e;
}

}  // namespace edgeneuro::mearm

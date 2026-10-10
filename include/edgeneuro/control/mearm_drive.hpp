#pragma once

#include "edgeneuro/control/mearm_calibration_data.hpp"
#include "edgeneuro/control/mearm_calibration_link.hpp"
#include "edgeneuro/control/mearm_real.hpp"
#include "edgeneuro/control/mearm_servo_maps.hpp"

namespace edgeneuro::mearm {

// Builds the compiled-in calibration (mearm_calibration_data.hpp) into `out`; false if Path B rejects it. Deliberately NOT
// a function-local static: a static initialized at run time needs a thread-safe guard (__cxa_guard_acquire), which drags
// abort() and the C++ unwinder into the bare-metal firmware and fails to link. Call it once at boot.
inline bool make_compiled_calibration(pathb::Calibration& out) {
    using namespace calibration_data;
    return pathb::Calibration::make({kHangX, kHangY, kHangZ}, {kForwardX, kForwardY, kForwardZ},
                                    {kLeftX, kLeftY, kLeftZ}, {kRightX, kRightY, kRightZ}, kZeroElbow, out);
}

// The compiled-in comfortable base reach (the calibration file's base_reach_*_raw ->
// gen_calibration_header.py), or the default mapping when none was measured (no tool captures that reach yet, so the
// default applies). False only if a compiled-in reach is unusable (then the caller should use the default).
inline bool make_compiled_base_reach(const pathb::Calibration& cal, real::BaseReach& out) {
    using namespace calibration_data;
    if (!kHasBaseReach) {
        out = real::default_base_reach(cal);
        return true;
    }
    return real::make_base_reach(cal, {kBaseReachLeftX, kBaseReachLeftY, kBaseReachLeftZ},
                                 {kBaseReachRightX, kBaseReachRightY, kBaseReachRightZ}, out);
}

// A calibration received over UART (mearm_calibration_link.hpp, 2026-10-03): built and validated exactly like the
// compiled-in one (pathb::Calibration::make, and make_base_reach for a measured reach). On success `cal`/`reach` are
// replaced; on any failure both are left untouched -- the arm keeps the calibration it had.
inline bool apply_calibration_message(const calibration_link::Values& v, pathb::Calibration& cal, real::BaseReach& reach) {
    pathb::Calibration c;
    if (!pathb::Calibration::make(v.hang, v.forward, v.left, v.right, v.zero_elbow, c)) return false;
    real::BaseReach r = real::default_base_reach(c);
    if (v.has_reach && !real::make_base_reach(c, v.reach_left, v.reach_right, r)) return false;
    cal = c;
    reach = r;
    return true;
}

namespace drive {

struct ArmCommand {
    unsigned base;
    unsigned shoulder;
    unsigned elbow;
    unsigned claw;
    bool hold;          // true: a sensor has failed -- write NOTHING, every servo keeps its current pulse (stops moving)
};

// Everything phase3_control_loop sends to the four real servos, before each servo's start-up ramp:
//   shoulder, elbow  real::pulses (Path B -> stretch -> measured angle lines -> measured envelope);
//   base             real::base_pulse (measured range 500..2500 us and direction, 2026-09-28);
//   claw             grip (0..1) through the measured claw map.
// cal == nullptr (a calibration that failed validation): base, shoulder and elbow hold rest -- never a guess.
// sensors_ok (edgeneuro::ImuHealth, 2026-09-28) is REQUIRED on purpose -- no default, so no caller can skip the check:
// false sets `hold`, and the caller must then leave every servo where it is (stop, do not move anywhere).
// reach: the base's comfortable reach (make_compiled_base_reach); nullptr = the default (unchanged) mapping.
inline ArmCommand command(const pathb::Calibration* cal, const pathb::Vec3& upper_raw, float elbow_bend, float grip,
                          bool sensors_ok, const real::BaseReach* reach = nullptr) {
    ArmCommand c{kBaseRestUs, kShoulderRestUs, kElbowRestUs, claw_map().pulse_us(grip), !sensors_ok};
    if (cal != nullptr) {
        const Pulses p = real::pulses(*cal, upper_raw, elbow_bend);
        c.shoulder = p.shoulder;
        c.elbow = p.elbow;
        c.base = reach ? real::base_pulse(*cal, upper_raw, *reach) : real::base_pulse(*cal, upper_raw);
    }
    return c;
}

// Which servos follow the sensors (2026-09-28: one at a time, to tell a bad servo / a weak supply apart). A masked-off
// servo is sent its rest pulse; hold is kept. The shoulder and elbow still go through joint_step, so e.g. the shoulder
// alone (elbow held at rest) stops where the envelope allows an elbow at rest.
constexpr unsigned kBaseBit = 1u, kShoulderBit = 2u, kElbowBit = 4u, kClawBit = 8u;
constexpr unsigned kAllServos = kBaseBit | kShoulderBit | kElbowBit | kClawBit;

inline ArmCommand only(ArmCommand c, unsigned mask) {
    if (!(mask & kBaseBit)) c.base = kBaseRestUs;
    if (!(mask & kShoulderBit)) c.shoulder = kShoulderRestUs;
    if (!(mask & kElbowBit)) c.elbow = kElbowRestUs;
    if (!(mask & kClawBit)) c.claw = kClawRestUs;
    return c;
}

// "R" (2026-10-03): run_demo_live.py --mearm asks the person to let the arm hang and, on Enter, sends R. request()
// puts every ramp back into its slow start-up rate; apply() then commands the start pose (the rest pulses) -- ignoring
// the sensors and any sensor hold, the start pose being known-safe -- until all four servos are exactly there, and from
// then on passes the arm's own command through again, the ramps re-armed so that following starts slowly too.
// Ramps: anything with .base/.shoulder/.elbow/.claw ServoStartupRamp members.
class Homing {
public:
    template <class Ramps>
    void request(Ramps& r) {
        active_ = true;
        rearm(r);
    }

    template <class Ramps>
    ArmCommand apply(const ArmCommand& sensors, Ramps& r) {
        if (!active_) return sensors;
        if (r.base.current_us() == kBaseRestUs && r.shoulder.current_us() == kShoulderRestUs &&
            r.elbow.current_us() == kElbowRestUs && r.claw.current_us() == kClawRestUs) {
            active_ = false;
            rearm(r);
            return sensors;
        }
        return {kBaseRestUs, kShoulderRestUs, kElbowRestUs, kClawRestUs, false};
    }

    bool active() const { return active_; }

private:
    template <class Ramps>
    static void rearm(Ramps& r) {
        r.base.rearm();
        r.shoulder.rearm();
        r.elbow.rearm();
        r.claw.rearm();
    }

    bool active_ = false;
};

}  // namespace drive
}  // namespace edgeneuro::mearm

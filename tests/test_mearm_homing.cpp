// "R" (2026-10-03, the author's design): run_demo_live.py --mearm asks the person to let the arm hang, and on Enter sends R;
// the firmware then walks all four servos back to the start pose (the rest pulses) at the SLOW start-up rate, ignoring
// the sensors meanwhile, and once there follows the arm again -- starting slowly. With the arm hanging the arm's own
// command is that same pose (9/13 calibration: hanging + straight elbow = 1500/1500/1500), so nothing jumps.
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_drive.hpp"
#include "edgeneuro/control/mearm_joint_step.hpp"
#include "edgeneuro/control/mearm_servo_maps.hpp"
#include "edgeneuro/control/servo_startup_ramp.hpp"

namespace M = edgeneuro::mearm;
namespace D = edgeneuro::mearm::drive;

namespace {
constexpr float kDt = 0.01f;

D::ArmCommand somewhere() { return {2200u, 1800u, 900u, 1450u, false}; }

struct Arm {
    edgeneuro::ServoStartupRamp base = M::base_ramp(), shoulder = M::shoulder_ramp(), elbow = M::elbow_ramp(),
                                claw = M::claw_ramp();
    // one firmware servo tick: homing decides the command, then the ramps (shoulder+elbow through joint_step)
    edgeneuro::Pulses step(D::Homing& h, const D::ArmCommand& sensors) {
        const D::ArmCommand c = h.apply(sensors, *this);
        b = base.step(static_cast<float>(c.base), kDt);
        const auto se = M::joint_step(shoulder, elbow, static_cast<float>(c.shoulder), static_cast<float>(c.elbow), kDt);
        k = claw.step(static_cast<float>(c.claw), kDt);
        s = se.shoulder;
        e = se.elbow;
        return se;
    }
    unsigned b = 0, s = 0, e = 0, k = 0;
};
}  // namespace

TEST_CASE("ServoStartupRamp::rearm puts a tracking ramp back into the slow start-up rate", "[mearm][homing]") {
    auto r = M::base_ramp();
    for (int i = 0; i < 2000; ++i) r.step(2000.0f, kDt);
    REQUIRE(r.tracking());
    r.rearm();
    REQUIRE_FALSE(r.tracking());
    const unsigned before = r.current_us();
    const unsigned after = r.step(1000.0f, kDt);
    REQUIRE(before - after <= static_cast<unsigned>(M::kStartRateUsPerS * kDt) + 1u);   // slow, not the fast rate
}

TEST_CASE("homing: without R the sensors' command passes straight through", "[mearm][homing]") {
    D::Homing h;
    Arm a;
    const auto c = h.apply(somewhere(), a);
    REQUIRE((c.base == 2200u && c.shoulder == 1800u && c.elbow == 900u && c.claw == 1450u));
    REQUIRE_FALSE(h.active());
}

TEST_CASE("homing: after R every servo walks back to the start pose slowly, whatever the arm does", "[mearm][homing]") {
    D::Homing h;
    Arm a;
    for (int i = 0; i < 3000; ++i) a.step(h, somewhere());               // tracking the arm somewhere else
    REQUIRE(a.b == 2200u);
    h.request(a);
    unsigned prev_b = a.b;
    bool reached = false;
    for (int i = 0; i < 2000 && !reached; ++i) {
        a.step(h, somewhere());                                           // the sensors still say "somewhere"
        REQUIRE(prev_b - a.b <= static_cast<unsigned>(M::kStartRateUsPerS * kDt) + 1u);   // slow walk back
        prev_b = a.b;
        reached = (a.b == M::kBaseRestUs && a.s == M::kShoulderRestUs && a.e == M::kElbowRestUs && a.k == M::kClawRestUs);
    }
    REQUIRE(reached);
}

TEST_CASE("homing: once at the start pose it follows the arm again, starting slowly", "[mearm][homing]") {
    D::Homing h;
    Arm a;
    for (int i = 0; i < 3000; ++i) a.step(h, somewhere());
    h.request(a);
    for (int i = 0; i < 3000 && h.active(); ++i) a.step(h, somewhere());
    REQUIRE_FALSE(h.active());
    const unsigned at_rest = a.b;
    a.step(h, somewhere());
    REQUIRE(a.b > at_rest);                                               // following again...
    REQUIRE(a.b - at_rest <= static_cast<unsigned>(M::kStartRateUsPerS * kDt) + 1u);   // ...slowly
}

TEST_CASE("homing ignores a sensor hold: the start pose is known-safe, it does not depend on the sensors", "[mearm][homing]") {
    D::Homing h;
    Arm a;
    for (int i = 0; i < 3000; ++i) a.step(h, somewhere());
    h.request(a);
    auto failed = somewhere();
    failed.hold = true;
    REQUIRE_FALSE(h.apply(failed, a).hold);
}

TEST_CASE("homing: a hanging arm's own command IS the start pose, so nothing moves after it", "[mearm][homing]") {
    // the 9/13 calibration's hanging pose with a straight elbow (see test_mearm_drive.cpp's compiled calibration)
    edgeneuro::mearm::pathb::Calibration cal;
    REQUIRE(M::make_compiled_calibration(cal));
    namespace C = edgeneuro::mearm::calibration_data;
    const auto cmd = D::command(&cal, {C::kHangX, C::kHangY, C::kHangZ}, C::kZeroElbow, 0.0f, true);
    REQUIRE((cmd.base == M::kBaseRestUs && cmd.shoulder == M::kShoulderRestUs && cmd.elbow == M::kElbowRestUs));
}

TEST_CASE("homing: it is not done until EVERY servo, the claw too, is at the start pose", "[mearm][homing]") {
    D::Homing h;
    Arm a;
    const D::ArmCommand claw_only{M::kBaseRestUs, M::kShoulderRestUs, M::kElbowRestUs, 1500u, false};
    for (int i = 0; i < 3000; ++i) a.step(h, claw_only);                  // only the claw is away from the start pose
    REQUIRE(a.k == 1500u);
    h.request(a);
    a.step(h, claw_only);
    REQUIRE(h.active());                                                  // the claw is still on its way back
    REQUIRE(a.k < 1500u);
}

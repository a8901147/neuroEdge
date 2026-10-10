// edgeneuro::mearm::drive: everything phase3_control_loop sends to the four real servos (before each servo's start-up
// ramp), in one host-testable place. Shoulder/elbow come from real::pulses with the compiled-in calibration; the base holds
// its rest pulse (its pulse <-> angle is not measured and its polarity is unverified); the claw comes from grip. A
// calibration that fails validation means shoulder and elbow hold rest -- never a guess.
#include <cmath>
#include <limits>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_drive.hpp"
#include "edgeneuro/control/mearm_joint_step.hpp"

namespace D = edgeneuro::mearm::drive;
namespace M = edgeneuro::mearm;
namespace C = edgeneuro::mearm::calibration_data;

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

const edgeneuro::mearm::pathb::Calibration* compiled() {
    static edgeneuro::mearm::pathb::Calibration cal;       // (test-side caching only)
    static const bool ok = M::make_compiled_calibration(cal);
    return ok ? &cal : nullptr;
}
}

TEST_CASE("the compiled-in calibration is valid and decodes its own HANG pose as hanging", "[mearm][drive]") {
    const auto* cal = compiled();
    REQUIRE(cal != nullptr);
    float tilt = -1.0f, az = 0.0f;
    REQUIRE(cal->decode({C::kHangX, C::kHangY, C::kHangZ}, tilt, az));
    REQUIRE(tilt < 1e-3f);
}

TEST_CASE("the compiled-in calibration keeps LEFT as LEFT: its own LEFT pose decodes to positive azimuth", "[mearm][drive]") {
    // (a mirrored calibration changes nothing while the base is held at rest -- shoulder/elbow only use the tilt and
    // |azimuth| -- but it would swing the base the wrong way the day the base is driven)
    const auto* cal = compiled();
    REQUIRE(cal != nullptr);
    float tilt = 0.0f, az_left = 0.0f, az_right = 0.0f;
    REQUIRE(cal->decode({C::kLeftX, C::kLeftY, C::kLeftZ}, tilt, az_left));
    REQUIRE(cal->decode({C::kRightX, C::kRightY, C::kRightZ}, tilt, az_right));
    REQUIRE(az_left > 0.1f);
    REQUIRE(az_right < -0.1f);
}

TEST_CASE("shoulder and elbow are exactly real::pulses with the compiled-in calibration", "[mearm][drive]") {
    const auto* cal = compiled();
    REQUIRE(cal != nullptr);
    const edgeneuro::mearm::pathb::Vec3 poses[] = {{C::kHangX, C::kHangY, C::kHangZ},
                                                    {C::kForwardX, C::kForwardY, C::kForwardZ},
                                                    {C::kLeftX, C::kLeftY, C::kLeftZ}};
    for (const auto& raw : poses) {
        for (float bend : {C::kZeroElbow, C::kZeroElbow + 1.0f, 2.8f}) {
            const auto cmd = D::command(cal, raw, bend, 0.0f, true);
            const auto p = M::real::pulses(*cal, raw, bend);
            REQUIRE((cmd.shoulder == p.shoulder && cmd.elbow == p.elbow));
        }
    }
}

TEST_CASE("the base follows real::base_pulse: LEFT turns it left (higher pulse, measured 2026-09-28), RIGHT right", "[mearm][drive]") {
    const auto* cal = compiled();
    REQUIRE(cal != nullptr);
    const edgeneuro::mearm::pathb::Vec3 left{C::kLeftX, C::kLeftY, C::kLeftZ}, right{C::kRightX, C::kRightY, C::kRightZ},
        hang{C::kHangX, C::kHangY, C::kHangZ};
    REQUIRE(D::command(cal, left, 0.5f, 0.3f, true).base == M::real::base_pulse(*cal, left));
    REQUIRE(D::command(cal, left, 0.5f, 0.3f, true).base > M::kBaseRestUs + 200u);
    REQUIRE(D::command(cal, right, 0.5f, 0.3f, true).base < M::kBaseRestUs - 200u);
    REQUIRE(D::command(cal, hang, 0.5f, 0.3f, true).base == M::kBaseRestUs);
}

TEST_CASE("the claw follows grip through the measured claw map", "[mearm][drive]") {
    const auto* cal = compiled();
    const edgeneuro::mearm::pathb::Vec3 hang{C::kHangX, C::kHangY, C::kHangZ};
    REQUIRE(D::command(cal, hang, C::kZeroElbow, 0.0f, true).claw == 1300u);       // relaxed = open = the claw's rest
    // gripping = 1500, which is also the claw's limit (the author, 2026-10-03; measured travel was 1300..1600)
    REQUIRE(D::command(cal, hang, C::kZeroElbow, 1.0f, true).claw == 1500u);
    REQUIRE(M::kClawHiUs == 1500u);
    const unsigned mid = D::command(cal, hang, C::kZeroElbow, kNaN, true).claw;    // a NaN grip never gives a wild pulse
    REQUIRE(mid >= 1300u);
    REQUIRE(mid <= 1500u);
}

TEST_CASE("without a valid calibration, shoulder, elbow and base hold rest and the claw still follows grip", "[mearm][drive]") {
    const auto cmd = D::command(nullptr, {C::kForwardX, C::kForwardY, C::kForwardZ}, 2.0f, 1.0f, true);
    REQUIRE(cmd.base == M::kBaseRestUs);
    REQUIRE(cmd.shoulder == M::kShoulderRestUs);
    REQUIRE(cmd.elbow == M::kElbowRestUs);
    REQUIRE(cmd.claw == 1500u);                       // a full grip (user, 2026-10-03)
}

TEST_CASE("every command is inside each servo's measured range and the shoulder/elbow envelope", "[mearm][drive]") {
    const auto* cal = compiled();
    const float vals[] = {-1.2f, -0.3f, 0.0f, 0.4f, 1.1f, kNaN};
    for (float x : vals) for (float y : vals) for (float z : vals) {
        for (float bend : {0.0f, 1.0f, 3.0f, kNaN}) {
            const auto c = D::command(cal, {x, y, z}, bend, 0.5f, true);
            const auto e = M::envelope().clamp(static_cast<float>(c.shoulder), static_cast<float>(c.elbow));
            REQUIRE((e.shoulder == c.shoulder && e.elbow == c.elbow));
            REQUIRE((c.base >= M::kBaseLoUs && c.base <= M::kBaseHiUs));
            REQUIRE((c.claw >= M::kClawLoUs && c.claw <= M::kClawHiUs));
        }
    }
}

// The firmware is bare metal: a function-local static initialized at RUN time needs __cxa_guard_acquire, which drags abort()
// and the C++ unwinder in and fails to link (found 2026-09-27 building phase3_control_loop with EDGENEURO_DRIVE_SERVOS=ON).
// So the tables behind the firmware's statics must be constant-initialized: their constructors must stay constexpr.
static_assert(edgeneuro::ServoAngleMap(0.0f, 1.0f, 1300u, 1600u).pulse_min_us() == 1300u,
              "ServoAngleMap must be constexpr-constructible (bare-metal statics)");
static_assert(edgeneuro::PulseEnvelope(edgeneuro::mearm::envelope_data::kShoulderUs,
                                       edgeneuro::mearm::envelope_data::kElbowLoUs,
                                       edgeneuro::mearm::envelope_data::kElbowHiUs,
                                       edgeneuro::mearm::envelope_data::kCount, 1500u, 1500u).shoulder_min() == 1500u,
              "PulseEnvelope must be constexpr-constructible (bare-metal statics)");

// ---- 2026-09-28: a failed sensor makes every servo HOLD (stop moving), it does not move the arm anywhere ----

TEST_CASE("with unhealthy sensors the command says hold, with healthy ones it does not", "[mearm][drive]") {
    const auto* cal = compiled();
    const edgeneuro::mearm::pathb::Vec3 hang{C::kHangX, C::kHangY, C::kHangZ};
    REQUIRE_FALSE(D::command(cal, hang, C::kZeroElbow, 0.0f, /*sensors_ok=*/true).hold);
    REQUIRE(D::command(cal, hang, C::kZeroElbow, 0.0f, /*sensors_ok=*/false).hold);
}

TEST_CASE("the default is: sensors assumed healthy only when the caller says so (no silent default)", "[mearm][drive]") {
    const auto* cal = compiled();
    const auto cmd = D::command(cal, {C::kHangX, C::kHangY, C::kHangZ}, C::kZeroElbow, 0.0f, false);
    REQUIRE(cmd.hold);
}

// 2026-09-28: to tell a bad servo / weak supply apart, the author drives ONE servo at a time from the sensors
// (EDGENEURO_SERVO_MASK, bit0 base, bit1 shoulder, bit2 elbow, bit3 claw); the others hold their rest pulse.
TEST_CASE("only(): a masked-off servo gets its rest pulse, the enabled ones are untouched, hold is kept", "[mearm][drive]") {
    const auto* cal = compiled();
    const edgeneuro::mearm::pathb::Vec3 left{C::kLeftX, C::kLeftY, C::kLeftZ};
    const auto full = D::command(cal, left, 0.5f, 0.8f, true);
    REQUIRE(D::only(full, D::kAllServos).base == full.base);
    REQUIRE(D::only(full, D::kAllServos).claw == full.claw);
    const auto s = D::only(full, D::kShoulderBit);
    REQUIRE(s.shoulder == full.shoulder);
    REQUIRE((s.base == M::kBaseRestUs && s.elbow == M::kElbowRestUs && s.claw == M::kClawRestUs));
    const auto b = D::only(full, D::kBaseBit);
    REQUIRE(b.base == full.base);
    REQUIRE((b.shoulder == M::kShoulderRestUs && b.elbow == M::kElbowRestUs && b.claw == M::kClawRestUs));
    REQUIRE(D::only(D::command(cal, left, 0.5f, 0.8f, false), D::kClawBit).hold);
}

TEST_CASE("shoulder alone, driven high: with the elbow held at 1500 the shoulder stops where that is still safe",
          "[mearm][drive]") {
    // since 2026-10-03 (height_reach) the person's ELBOW BEND drives the shoulder servo: fully bent -> its top
    const auto* cal = compiled();
    const edgeneuro::mearm::pathb::Vec3 hang{C::kHangX, C::kHangY, C::kHangZ};
    const auto cmd = D::only(D::command(cal, hang, C::kZeroElbow + edgeneuro::mearm::pathb::kElbowSwingRad, 0.0f, true),
                             D::kShoulderBit);
    REQUIRE(cmd.shoulder > 1800u);                   // what the sensors ask for is above where elbow 1500 is safe
    auto sr = M::shoulder_ramp();
    auto er = M::elbow_ramp();
    edgeneuro::Pulses p{};
    for (int i = 0; i < 1000; ++i) {
        p = M::joint_step(sr, er, static_cast<float>(cmd.shoulder), static_cast<float>(cmd.elbow), 0.01f);
        const auto c = M::envelope().clamp(static_cast<float>(p.shoulder), static_cast<float>(p.elbow));
        REQUIRE((c.shoulder == p.shoulder && c.elbow == p.elbow));
    }
    REQUIRE(p.elbow == M::kElbowRestUs);
    REQUIRE(p.shoulder >= 1800u);                    // it still rises as far as is safe
}

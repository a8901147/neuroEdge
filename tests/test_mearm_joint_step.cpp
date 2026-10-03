// mearm::joint_step: the shoulder and elbow ramps step TOGETHER through the measured safe envelope, so the real arm is
// inside it on every tick -- not only at the target. 2026-09-28: with the envelope extended to shoulder 2100 (where the
// elbow may only go up to 1000), two independent ramps could briefly pass through e.g. (2025, 1300) on a fast raise,
// where the elbow's linkage hits the upper arm.
#include <cstdlib>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_envelope.hpp"
#include "edgeneuro/control/mearm_joint_step.hpp"
#include "edgeneuro/control/mearm_servo_maps.hpp"

namespace M = edgeneuro::mearm;

namespace {
constexpr float kDt = 0.01f;   // the firmware's servo cadence (every 10th 1 kHz tick)

bool inside(unsigned s, unsigned e) {
    const auto p = M::envelope().clamp(static_cast<float>(s), static_cast<float>(e));
    return p.shoulder == s && p.elbow == e;
}

struct Arm {
    edgeneuro::ServoStartupRamp shoulder = M::shoulder_ramp();
    edgeneuro::ServoStartupRamp elbow = M::elbow_ramp();
};

// Walk both ramps to (s, e) with plain independent steps (the rest->tracking start-up done), for a starting pose.
void settle(Arm& a, float s, float e) {
    for (int i = 0; i < 2000; ++i) M::joint_step(a.shoulder, a.elbow, s, e, kDt);
}

// the feasible targets: every measured shoulder with elbows across its window
std::vector<edgeneuro::Pulses> targets() {
    std::vector<edgeneuro::Pulses> out;
    namespace D = M::envelope_data;
    for (unsigned i = 0; i < D::kCount; ++i)
        for (unsigned e = D::kElbowLoUs[i]; e <= D::kElbowHiUs[i]; e += 125u) out.push_back({D::kShoulderUs[i], e});
    return out;
}
}  // namespace

TEST_CASE("joint_step: every tick is inside the envelope, for every move between feasible poses", "[mearm][joint]") {
    const auto ts = targets();
    REQUIRE(ts.size() >= 40u);
    for (const auto& from : ts) {
        for (const auto& to : ts) {
            Arm a;
            settle(a, static_cast<float>(from.shoulder), static_cast<float>(from.elbow));
            for (int i = 0; i < 400; ++i) {
                const auto p = M::joint_step(a.shoulder, a.elbow, static_cast<float>(to.shoulder),
                                             static_cast<float>(to.elbow), kDt);
                INFO("from (" << from.shoulder << "," << from.elbow << ") to (" << to.shoulder << "," << to.elbow
                              << ") tick " << i << ": (" << p.shoulder << "," << p.elbow << ")");
                REQUIRE(inside(p.shoulder, p.elbow));
            }
        }
    }
}

TEST_CASE("joint_step: it always gets there (no deadlock) within a couple of seconds", "[mearm][joint]") {
    const auto ts = targets();
    for (const auto& from : ts) {
        for (const auto& to : ts) {
            Arm a;
            settle(a, static_cast<float>(from.shoulder), static_cast<float>(from.elbow));
            edgeneuro::Pulses p{};
            for (int i = 0; i < 300; ++i)            // 3 s of servo ticks
                p = M::joint_step(a.shoulder, a.elbow, static_cast<float>(to.shoulder), static_cast<float>(to.elbow), kDt);
            INFO("from (" << from.shoulder << "," << from.elbow << ") to (" << to.shoulder << "," << to.elbow << ")");
            REQUIRE(p.shoulder == to.shoulder);
            REQUIRE(p.elbow == to.elbow);
        }
    }
}

TEST_CASE("joint_step: the fast straight-arm raise that motivated it -- independent ramps leave the envelope, it does not",
          "[mearm][joint]") {
    // mid-shoulder with the elbow high (1650, 1800) -> raised with the elbow pulled in (2100, 1000)
    edgeneuro::ServoStartupRamp s = M::shoulder_ramp(), e = M::elbow_ramp();
    Arm a;
    settle(a, 1650.0f, 1800.0f);
    for (int i = 0; i < 2000; ++i) {                 // the same start for the naive pair
        s.step(1650.0f, kDt);
        e.step(1800.0f, kDt);
    }
    bool naive_left = false;
    for (int i = 0; i < 100; ++i) {
        const unsigned ns = s.step(2100.0f, kDt), ne = e.step(1000.0f, kDt);
        naive_left = naive_left || !inside(ns, ne);
        const auto p = M::joint_step(a.shoulder, a.elbow, 2100.0f, 1000.0f, kDt);
        REQUIRE(inside(p.shoulder, p.elbow));
    }
    REQUIRE(naive_left);                             // the guard is real: without it this move leaves the envelope
}

TEST_CASE("joint_step: where the envelope does not bind, it is exactly the two plain ramps", "[mearm][joint]") {
    edgeneuro::ServoStartupRamp s = M::shoulder_ramp(), e = M::elbow_ramp();
    Arm a;
    for (int i = 0; i < 1000; ++i) {                 // shoulder 1500 -> 1650 and elbow 1500 -> 800: never near an edge
        const unsigned ns = s.step(1650.0f, kDt), ne = e.step(800.0f, kDt);
        const auto p = M::joint_step(a.shoulder, a.elbow, 1650.0f, 800.0f, kDt);
        REQUIRE(p.shoulder == ns);
        REQUIRE(p.elbow == ne);
    }
}

TEST_CASE("joint_step: even a target outside the envelope never takes the arm out of it -- it waits at the edge",
          "[mearm][joint]") {
    // drive::command clamps its targets, but this is the last line of defence: e.g. from (1950, 1425) toward (2100, 1850)
    // no single step is allowed (both together, elbow alone, shoulder alone all leave the envelope) -> both wait
    Arm a;
    settle(a, 1950.0f, 1425.0f);
    for (int i = 0; i < 300; ++i) {
        const auto p = M::joint_step(a.shoulder, a.elbow, 2100.0f, 1850.0f, kDt);
        REQUIRE(inside(p.shoulder, p.elbow));
    }
}

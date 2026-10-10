// The base follows SLOWLY while the arm is being raised or lowered (2026-10-04, the author's choice). Recorded on the real
// arm the same day (data/base_raise_20261004.csv): held still the base is steady (67 us over 8 s), but on the way up or
// down -- above all just after leaving the hanging pose, where the arm's azimuth is very sensitive to a small sideways
// offset -- the base target jumped (raise forward 1152..1739 us, raise left-front down to 533). "Raising" is decided by
// the DIRECTION of the motion (more up/down than sideways), not by its speed, so a slow raise still counts (the author's
// point); only sensor noise is ignored. Not raising, the base follows at once, so it ends where it always would.
//
// The real recording is fed through the firmware's whole servo-block pipeline (1-euro filter -> drive::command ->
// hysteresis -> this) at its 10 ms cadence; asserts are properties with headroom (CLAUDE.md), not pinned values.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_drive.hpp"
#include "edgeneuro/control/mearm_input_filter.hpp"

namespace M = edgeneuro::mearm;
namespace D = edgeneuro::mearm::drive;
using V = edgeneuro::mearm::pathb::Vec3;

namespace {
constexpr float kDt = 0.01f;   // the servo block's cadence

const M::pathb::Calibration& cal() {
    static M::pathb::Calibration c;
    static const bool ok = M::make_compiled_calibration(c);
    REQUIRE(ok);
    return c;
}

struct Sample {
    double t;
    V up;
};

const std::map<std::string, std::vector<Sample>>& recording() {
    static std::map<std::string, std::vector<Sample>> phases;
    if (phases.empty()) {
        std::ifstream in(std::string(EDGENEURO_DATA_DIR) + "/base_raise_20261004.csv");
        REQUIRE(in.good());
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#' || line.rfind("phase,", 0) == 0) continue;
            std::stringstream ss(line);
            std::string name, f;
            Sample s{};
            std::getline(ss, name, ',');
            std::getline(ss, f, ',');
            s.t = std::stod(f);
            for (int i = 0; i < 3; ++i) {
                std::getline(ss, f, ',');
                s.up[static_cast<unsigned>(i)] = std::stof(f);
            }
            phases[name].push_back(s);
        }
    }
    return phases;
}

struct Run {
    std::vector<unsigned> before;   // today's pipeline (no slow follow)
    std::vector<unsigned> after;    // with BaseRaiseFollow
};

// The recording (~30 samples/s) at the servo block's 10 ms cadence (each reading held until the next one, as the
// firmware holds its latest IMU reading); `slower` stretches time (the same path, raised that many times slower);
// `hold_s` keeps the last reading for that long after the phase (the arm stopped).
Run run(const std::string& phase, double slower = 1.0, double hold_s = 0.0) {
    const auto& ss = recording().at(phase);
    M::ArmInputFilter filter;
    M::Hysteresis hold;
    M::BaseRaiseFollow follow;
    Run r;
    std::size_t i = 0;
    const double end = ss.back().t * slower + hold_s;
    for (double t = ss.front().t * slower; t <= end; t += kDt) {
        while (i + 1 < ss.size() && ss[i + 1].t * slower <= t) ++i;
        const auto f = filter.update(ss[i].up, ss[i].up, kDt);
        const auto cmd = D::command(&cal(), f.upper, f.elbow_bend, 0.0f, true);
        const unsigned target = hold.apply(cmd.base, M::kBaseHysteresisUs);
        r.before.push_back(target);
        r.after.push_back(follow.apply(target, &cal(), f.upper, kDt));
    }
    return r;
}

unsigned span(const std::vector<unsigned>& v) {
    const auto mm = std::minmax_element(v.begin(), v.end());
    return *mm.second - *mm.first;
}
double mean_gap(const Run& r) {
    double s = 0;
    for (std::size_t k = 0; k < r.before.size(); ++k)
        s += std::fabs(static_cast<double>(r.after[k]) - static_cast<double>(r.before[k]));
    return s / static_cast<double>(r.before.size());
}
}  // namespace

TEST_CASE("real raises: the base swings much less on the way up and down", "[mearm][base_follow]") {
    // simulated on this recording: forward 587 -> ~160 us, left-front 970 -> ~400 us
    for (const char* phase : {"raise_forward", "raise_left_front"}) {
        INFO(phase);
        const Run r = run(phase);
        CHECK(span(r.after) <= span(r.before) * 6 / 10);
    }
}

TEST_CASE("a slow raise still counts as raising (decided by direction, not speed)", "[mearm][base_follow]") {
    // the author's point: the same real path raised 3x and 5x slower must still be calmed
    for (double slower : {3.0, 5.0}) {
        for (const char* phase : {"raise_forward", "raise_left_front"}) {
            INFO(phase << " " << slower << "x slower");
            const Run r = run(phase, slower);
            CHECK(span(r.after) <= span(r.before) * 8 / 10);
        }
    }
}

TEST_CASE("a real sideways swing is followed as before", "[mearm][base_follow]") {
    const Run r = run("swing_only");
    CHECK(mean_gap(r) <= 20.0);
    CHECK(span(r.after) * 10 >= span(r.before) * 9);
}

TEST_CASE("a still arm: the base is exactly as before", "[mearm][base_follow]") {
    const Run r = run("hold_forward");
    CHECK(r.after == r.before);
}

TEST_CASE("once the arm stops, the base ends exactly where it always would", "[mearm][base_follow]") {
    for (const char* phase : {"raise_forward", "raise_left_front", "raise_right_front", "swing_only", "check_diagonal"}) {
        INFO(phase);
        const Run r = run(phase, 1.0, 1.0);
        CHECK(r.after.back() == r.before.back());
    }
}

TEST_CASE("check (not used to choose the numbers): the author's lower-left -> upper-right raise loses its spike",
          "[mearm][base_follow]") {
    const Run r = run("check_diagonal");
    CHECK(span(r.after) <= span(r.before) * 8 / 10);
}

TEST_CASE("the first value passes as is", "[mearm][base_follow]") {
    M::BaseRaiseFollow f;
    REQUIRE(f.apply(1234u, &cal(), cal().h, kDt) == 1234u);
}

TEST_CASE("no calibration or no valid reading: the target passes as is", "[mearm][base_follow]") {
    M::BaseRaiseFollow f;
    REQUIRE(f.apply(900u, nullptr, cal().h, kDt) == 900u);
    REQUIRE(f.apply(2100u, &cal(), V{0.0f, 0.0f, 0.0f}, kDt) == 2100u);
}

namespace {
// a unit vector `tilt_deg` from HANG towards FORWARD (azimuth 0), turned `az_deg` about HANG
V pose(float tilt_deg, float az_deg) {
    const auto& c = cal();
    const float t = tilt_deg * 3.14159265f / 180.0f, a = az_deg * 3.14159265f / 180.0f;
    V v{};
    for (unsigned i = 0; i < 3; ++i)
        v[i] = c.h[i] * std::cos(t) + std::sin(t) * (c.e1[i] * std::cos(a) + c.e2[i] * std::sin(a));
    return v;
}
}  // namespace

TEST_CASE("while raising, the base moves at most kRaiseFollowUsPerS; when the raise stops it catches up at once",
          "[mearm][base_follow]") {
    M::BaseRaiseFollow f;
    float tilt = 30.0f;
    unsigned out = f.apply(1500u, &cal(), pose(tilt, 0.0f), kDt);
    for (int k = 0; k < 50; ++k) {   // 0.5 s raising at 40 deg/s straight up while the target says "far right"
        tilt += 40.0f * kDt;
        out = f.apply(800u, &cal(), pose(tilt, 0.0f), kDt);
    }
    // seen as raising from the very next step (the motion is compared with the readings of the last window, every
    // step), so the base moved at the slow rate the whole time -- not frozen, not jumped
    const float slow = M::kRaiseFollowUsPerS * 0.5f;
    CHECK(static_cast<float>(out) >= 1500.0f - slow - 1.0f);
    CHECK(static_cast<float>(out) <= 1500.0f - slow + 1.0f + M::kRaiseFollowUsPerS * kDt);
    for (int k = 0; k < 30; ++k) out = f.apply(800u, &cal(), pose(tilt, 0.0f), kDt);   // stopped
    CHECK(out == 800u);
}

TEST_CASE("a sideways swing is not slowed down", "[mearm][base_follow]") {
    M::BaseRaiseFollow f;
    float az = 0.0f;
    f.apply(1500u, &cal(), pose(70.0f, az), kDt);
    unsigned out = 0;
    for (int k = 0; k < 50; ++k) {
        az += 30.0f * kDt;
        out = f.apply(1500u + static_cast<unsigned>(k) * 20u, &cal(), pose(70.0f, az), kDt);
    }
    CHECK(out == 1500u + 49u * 20u);
}

TEST_CASE("after a reading that is not usable, the next one passes as is (no history from before it)",
          "[mearm][base_follow]") {
    M::BaseRaiseFollow f;
    float tilt = 30.0f;
    f.apply(1500u, &cal(), pose(tilt, 0.0f), kDt);
    for (int k = 0; k < 20; ++k) {   // raising
        tilt += 40.0f * kDt;
        f.apply(1500u, &cal(), pose(tilt, 0.0f), kDt);
    }
    REQUIRE(f.apply(1500u, &cal(), V{0.0f, 0.0f, 0.0f}, kDt) == 1500u);
    tilt += 40.0f * kDt;
    CHECK(f.apply(800u, &cal(), pose(tilt, 0.0f), kDt) == 800u);
}

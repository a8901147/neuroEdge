// The MEArm's input conditioning (2026-10-03): the servos run at full speed (the ramp is only a safety limit) and the
// COMMAND is made steady instead -- a 1-euro filter on both arm IMUs' raw accel vectors, then a small hysteresis on the
// base target. Measured on the real arm the same day: holding the arm forward, the base swung over 383 us (std 102 us)
// while shoulder and elbow stayed within ~3 us -- the arm's own sway (about 30 us of base per degree of arm azimuth),
// not sensor noise (the MPU6050's 5 Hz DLPF already removes that). Checked here with that sway simulated: steadier at
// rest, and a real swing still arrives promptly.
#include <cmath>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_drive.hpp"
#include "edgeneuro/control/mearm_input_filter.hpp"

namespace M = edgeneuro::mearm;
namespace D = edgeneuro::mearm::drive;
namespace C = edgeneuro::mearm::calibration_data;
using V = edgeneuro::mearm::pathb::Vec3;

namespace {
constexpr float kDt = 0.01f;   // the servo block's cadence
constexpr float kPi = 3.14159265f;

V unit(V v) {
    const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return {v[0] / n, v[1] / n, v[2] / n};
}
// Rodrigues rotation of v about the (unit) axis k
V rotate(V v, V k, float a) {
    k = unit(k);
    const float c = std::cos(a), s = std::sin(a), d = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
    const V x{k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]};
    V r{};
    for (int i = 0; i < 3; ++i) r[i] = v[i] * c + x[i] * s + k[i] * d * (1 - c);
    return r;
}
const V kHang{C::kHangX, C::kHangY, C::kHangZ};
const V kForward{C::kForwardX, C::kForwardY, C::kForwardZ};
const V kLeft{C::kLeftX, C::kLeftY, C::kLeftZ};
// a forearm vector at a fixed bend from the upper arm (straight elbow = the same direction)
V forearm_like(V upper) { return upper; }

const M::pathb::Calibration& cal() {
    static M::pathb::Calibration c;
    static const bool ok = M::make_compiled_calibration(c);
    REQUIRE(ok);
    return c;
}

double stdev(const std::vector<double>& v) {
    double m = 0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size()));
}

struct Pipeline {   // what the firmware does each servo tick, without the ramps
    M::ArmInputFilter filter;
    M::Hysteresis base_hold;
    unsigned base(V up, V fore) {
        const auto f = filter.update(up, fore, kDt);
        const auto cmd = D::command(&cal(), f.upper, f.elbow_bend, 0.0f, true);
        return base_hold.apply(cmd.base, M::kBaseHysteresisUs);
    }
};
unsigned raw_base(V up) { return D::command(&cal(), up, C::kZeroElbow, 0.0f, true).base; }
}  // namespace

TEST_CASE("elbow bend between two vectors: same direction 0, perpendicular pi/2, a missing reading 0", "[mearm][input_filter]") {
    REQUIRE(std::fabs(M::elbow_bend_between({0, 0, 1}, {0, 0, 2})) < 1e-6f);
    REQUIRE(std::fabs(M::elbow_bend_between({1, 0, 0}, {0, 1, 0}) - kPi / 2) < 1e-5f);
    REQUIRE(M::elbow_bend_between({0, 0, 0}, {0, 1, 0}) == 0.0f);
    REQUIRE(M::elbow_bend_between({1, 0, 0}, {-1, 0, 0}) > 3.14f);   // clamped, never NaN
}

TEST_CASE("before any reading the filter passes the all-zero 'no reading yet' vector, which the drive answers with rest",
          "[mearm][input_filter]") {
    M::ArmInputFilter f;
    const auto out = f.update({0, 0, 0}, {0, 0, 0}, kDt);
    REQUIRE((out.upper[0] == 0.0f && out.upper[1] == 0.0f && out.upper[2] == 0.0f));
    REQUIRE(D::command(&cal(), out.upper, out.elbow_bend, 0.0f, true).base == M::kBaseRestUs);
}

TEST_CASE("hysteresis: holds inside the band, follows beyond it, takes the first value as is", "[mearm][input_filter]") {
    M::Hysteresis h;
    REQUIRE(h.apply(1500u, 10u) == 1500u);
    REQUIRE(h.apply(1508u, 10u) == 1500u);
    REQUIRE(h.apply(1492u, 10u) == 1500u);
    REQUIRE(h.apply(1511u, 10u) == 1511u);
    REQUIRE(h.apply(1400u, 10u) == 1400u);
}

TEST_CASE("arm held forward: the fast jitter the user saw is gone, the slow drift (real motion) still passes",
          "[mearm][input_filter]") {
    // Modelled on the real capture (data/servo_response_20261003-135305.json, "held forward"): the base's own wander was
    // almost all below 1 Hz (std 100.8 us of slow drift -- the arm really drifting a few degrees), on top of fast jitter
    // (std 10.6 us inside 0.2 s windows, single steps up to 45 us) -- the "fast shaking" the user reported. At ~30 us of
    // base per degree of arm azimuth: drift ~ +-3 deg at 0.1-0.3 Hz, jitter ~0.35 deg per sample.
    Pipeline p;
    unsigned seed = 12345u;
    auto noise = [&seed]() {                              // deterministic, roughly normal (sum of uniforms), std ~1
        float acc = 0.0f;
        for (int k = 0; k < 12; ++k) {
            seed = seed * 1664525u + 1013904223u;
            acc += static_cast<float>(seed >> 8) / 16777216.0f;
        }
        return acc - 6.0f;
    };
    std::vector<double> raw, filtered;
    for (int i = 0; i < 2000; ++i) {
        const float t = static_cast<float>(i) * kDt;
        const float drift = 2.5f * std::sin(2 * kPi * 0.12f * t) + 1.0f * std::sin(2 * kPi * 0.27f * t);
        const float az = (drift + 0.35f * noise()) * kPi / 180.0f;
        const V up = rotate(kForward, kHang, az);
        const unsigned b = p.base(up, forearm_like(up));
        if (i >= 300) {
            raw.push_back(raw_base(up));
            filtered.push_back(b);
        }
    }
    // split each series like the analysis did: slow = 0.2 s moving average, fast = what is left
    auto split = [](const std::vector<double>& v, std::vector<double>& slow, std::vector<double>& fast) {
        const int w = 20;
        for (std::size_t i = w; i + w < v.size(); ++i) {
            double m = 0;
            for (int k = -w / 2; k < w / 2; ++k) m += v[i + k];
            m /= w;
            slow.push_back(m);
            fast.push_back(v[i] - m);
        }
    };
    std::vector<double> rs, rf, fs, ff;
    split(raw, rs, rf);
    split(filtered, fs, ff);
    INFO("fast jitter: raw " << stdev(rf) << " us -> filtered " << stdev(ff) << " us; slow drift: raw " << stdev(rs)
                             << " us -> filtered " << stdev(fs) << " us");
    REQUIRE(stdev(rf) > 6.0);                             // the simulated jitter is of the measured size
    REQUIRE(stdev(ff) * 3.0 < stdev(rf));                 // the fast shaking is cut at least 3x
    REQUIRE(stdev(fs) > 0.5 * stdev(rs));                 // the slow drift is real motion and still passes
}

TEST_CASE("a real swing (forward -> left in 0.5 s) still arrives promptly", "[mearm][input_filter]") {
    Pipeline p;
    for (int i = 0; i < 300; ++i) p.base(kForward, kForward);   // settled, held forward
    const unsigned target = raw_base(kLeft);
    int reached_at = -1;
    for (int i = 0; i < 300; ++i) {
        const float f = std::fmin(1.0f, static_cast<float>(i) * kDt / 0.5f);
        V mix{};
        for (int k = 0; k < 3; ++k) mix[k] = kForward[k] + f * (kLeft[k] - kForward[k]);
        const V up = unit(mix);
        const unsigned b = p.base(up, up);
        if (reached_at < 0 && std::fabs(static_cast<double>(b) - static_cast<double>(target)) <= 25.0) reached_at = i;
    }
    INFO("reached within 25 us of the target after " << reached_at * 10 << " ms (the motion itself takes 500 ms)");
    REQUIRE(reached_at >= 0);
    REQUIRE(reached_at * kDt <= 0.5f + 0.15f);             // at most ~150 ms behind the end of the motion
}

TEST_CASE("hysteresis: a move of exactly the band is still held", "[mearm][input_filter]") {
    M::Hysteresis h;
    h.apply(1500u, 10u);
    REQUIRE(h.apply(1510u, 10u) == 1500u);
    REQUIRE(h.apply(1490u, 10u) == 1500u);
}

TEST_CASE("elbow bend of identical vectors is 0 even when float rounding puts the cosine just past 1", "[mearm][input_filter]") {
    const V a{0.049999997f, 0.701499999f, 0.299500018f};   // found by search: (a.a)/(|a||a|) rounds to 1.00000012f
    const float b = M::elbow_bend_between(a, a);
    REQUIRE_FALSE(std::isnan(b));
    REQUIRE(b == 0.0f);
}

TEST_CASE("the elbow bend is taken between the two FILTERED vectors, so upper-arm jitter does not shake the elbow",
          "[mearm][input_filter]") {
    M::ArmInputFilter f;
    unsigned seed = 7u;
    auto noise = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return (static_cast<float>(seed >> 8) / 16777216.0f - 0.5f) * 0.02f;   // +-0.01 g
    };
    const V fore{0.0f, 0.6f, 0.8f};
    std::vector<double> raw_bend, filt_bend;
    for (int i = 0; i < 1000; ++i) {
        const V up{kForward[0] + noise(), kForward[1] + noise(), kForward[2] + noise()};
        const auto out = f.update(up, fore, kDt);
        if (i >= 200) {
            raw_bend.push_back(M::elbow_bend_between(up, fore));
            filt_bend.push_back(out.elbow_bend);
        }
    }
    REQUIRE(stdev(filt_bend) * 3.0 < stdev(raw_bend));
}

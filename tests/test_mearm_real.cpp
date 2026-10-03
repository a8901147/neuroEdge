// C++ port of tools/mujoco_bridge/mearm_real.py (stretch mode): upper-arm gravity vector + elbow reading -> REAL MEArm
// (shoulder, elbow) servo pulses, through Path B's decode, the measured link-angle lines and the measured safe envelope.
// Python is the source of truth: data/real_golden.csv is generated from it (gen_real_golden.py, stale -> test_real_golden.py
// fails). Float vs double can land a value on the other side of a .5 rounding boundary, so parity allows 1 us -- and it
// is checked that this only happens rarely.
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_real.hpp"

namespace P = edgeneuro::mearm::pathb;
namespace R = edgeneuro::mearm::real;

namespace {
std::vector<std::vector<float>> read_csv(const std::string& name) {
    std::ifstream f(std::string(EDGENEURO_DATA_DIR) + "/" + name);
    REQUIRE(f.is_open());
    std::string line;
    std::getline(f, line);
    std::vector<std::vector<float>> rows;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string cell;
        std::vector<float> row;
        while (std::getline(ss, cell, ',')) row.push_back(std::stof(cell));
        rows.push_back(row);
    }
    return rows;
}

P::Calibration calibration() {
    const auto r = read_csv("pathb_calibration.csv").at(0);
    P::Calibration cal;
    REQUIRE(P::Calibration::make({r[0], r[1], r[2]}, {r[3], r[4], r[5]}, {r[6], r[7], r[8]}, {r[9], r[10], r[11]},
                                 r[12], cal));
    return cal;
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
}  // namespace

TEST_CASE("C++ real-arm pulses reproduce the Python golden table within 1 us, and exactly almost always", "[mearm][real]") {
    const auto cal = calibration();
    const auto rows = read_csv("real_golden.csv");
    REQUIRE(rows.size() >= 400u);
    std::size_t off_by_one = 0;
    for (const auto& r : rows) {
        const auto p = R::pulses(cal, {r[0], r[1], r[2]}, r[3]);
        INFO("raw=(" << r[0] << "," << r[1] << "," << r[2] << ") bend=" << r[3]);
        const long ds = std::labs(static_cast<long>(p.shoulder) - static_cast<long>(r[4]));
        const long de = std::labs(static_cast<long>(p.elbow) - static_cast<long>(r[5]));
        const long db = std::labs(static_cast<long>(R::base_pulse(cal, {r[0], r[1], r[2]})) - static_cast<long>(r[6]));
        REQUIRE(ds <= 1);
        REQUIRE(de <= 1);
        REQUIRE(db <= 1);
        off_by_one += (ds + de + db) > 0 ? 1u : 0u;
    }
    REQUIRE(off_by_one * 50u <= rows.size());          // at most 2% of rows touched by float rounding
}

TEST_CASE("every real-arm output is inside the measured envelope, for any input", "[mearm][real]") {
    const auto cal = calibration();
    const auto& env = edgeneuro::mearm::envelope();
    const float raws[][3] = {{0.99f, 0.05f, 0.27f}, {0.03f, -0.01f, 1.02f}, {0.12f, 0.45f, 0.92f}, {-1.0f, 0.0f, 0.0f},
                             {0.0f, 0.0f, 0.0f},    {kNaN, 0.0f, 1.0f},     {kInf, 0.0f, 1.0f}};
    for (const auto& raw : raws) {
        for (float bend : {0.0f, 0.43f, 1.5f, 3.1f, kNaN, kInf, -kInf}) {
            const auto p = R::pulses(cal, {raw[0], raw[1], raw[2]}, bend);
            const auto c = env.clamp(static_cast<float>(p.shoulder), static_cast<float>(p.elbow));
            INFO("raw=(" << raw[0] << "," << raw[1] << "," << raw[2] << ") bend=" << bend);
            REQUIRE((c.shoulder == p.shoulder && c.elbow == p.elbow));
        }
    }
}

TEST_CASE("an invalid upper-arm reading gives the rest shoulder pulse", "[mearm][real]") {
    const auto cal = calibration();
    for (auto raw : {P::Vec3{0.f, 0.f, 0.f}, P::Vec3{kNaN, 0.f, 1.f}, P::Vec3{kInf, 0.f, 1.f}}) {
        REQUIRE(R::pulses(cal, raw, 0.43f).shoulder == edgeneuro::mearm::kShoulderRestUs);
        // whatever the elbow says (height_reach: the bend would otherwise drive the shoulder servo)
        const auto bent = R::pulses(cal, raw, 0.43f + P::kElbowSwingRad);
        REQUIRE((bent.shoulder == edgeneuro::mearm::kShoulderRestUs && bent.elbow == edgeneuro::mearm::kElbowRestUs));
    }
}

TEST_CASE("height_reach (2026-10-03): hanging + straight is the start pose; raising lowers the ELBOW servo, bending "
          "raises the SHOULDER servo", "[mearm][real]") {
    const auto cal = calibration();
    const auto r = read_csv("pathb_calibration.csv").at(0);
    const P::Vec3 hang{r[0], r[1], r[2]}, fwd{r[3], r[4], r[5]};
    const float straight = r[12], bent = r[12] + P::kElbowSwingRad;   // zero_elbow + the measured swing
    const auto start = R::pulses(cal, hang, straight);
    REQUIRE((start.shoulder == 1500u && start.elbow == 1500u));
    const auto raised = R::pulses(cal, fwd, straight);
    REQUIRE(raised.shoulder == 1500u);                         // raising leaves the shoulder (reach) servo alone
    REQUIRE(raised.elbow + 300u < start.elbow);                // ...and lowers the elbow (height) servo
    const auto bent_hang = R::pulses(cal, hang, bent);
    REQUIRE(bent_hang.shoulder ==                              // bending: the shoulder servo to the end of its range
            edgeneuro::mearm::envelope_data::kShoulderUs[edgeneuro::mearm::envelope_data::kCount - 1u]);
}

TEST_CASE("model command -> pulse follows the measured lines (independent literals)", "[mearm][real]") {
    // upper arm: 50.1 deg at 1500, +0.067 deg/us; forearm: -54.9 deg at 1500, -0.082 deg/us (SESSION_LOG 2026-09-27)
    const auto p = R::pulses_from_model_ctrl(0.6957f, 1.8327f);       // the rest pose's model command, measured
    REQUIRE(std::fabs(p.shoulder - 1500.0f) < 2.0f);
    REQUIRE(std::fabs(p.elbow - 1500.0f) < 2.0f);
    const auto q = R::pulses_from_model_ctrl(0.6957f - 0.1f, 1.8327f);  // shoulder command smaller = upper arm higher
    REQUIRE(q.shoulder > p.shoulder);
}

TEST_CASE("height_reach: a small change never makes the elbow servo jump (the envelope's stepped window is smoothed)",
          "[mearm][real]") {
    // 2026-10-03 real arm: one degree of elbow bend around 16 deg moved the elbow servo up to 350 us (the envelope's
    // window steps at its measured shoulder positions). Same scan as test_mearm_real.py's HeightReachTest.
    const auto cal = calibration();
    const auto r = read_csv("pathb_calibration.csv").at(0);
    const P::Vec3 hang{r[0], r[1], r[2]};
    unsigned prev = R::pulses(cal, hang, r[12]).elbow;
    for (int i = 1; i < 260; ++i) {
        const unsigned e = R::pulses(cal, hang, r[12] + static_cast<float>(i) * 0.5f * 3.14159265f / 180.0f).elbow;
        INFO("bend " << i * 0.5f << " deg: " << prev << " -> " << e);
        REQUIRE(std::labs(static_cast<long>(e) - static_cast<long>(prev)) <= 25);
        prev = e;
    }
}

TEST_CASE("the smoothed elbow window never leaves the measured envelope", "[mearm][real]") {
    const auto& env = edgeneuro::mearm::envelope();
    for (unsigned s = env.shoulder_min(); s <= env.shoulder_max(); ++s) {
        const auto w = R::smooth_elbow_window(static_cast<float>(s));
        INFO("shoulder " << s);
        REQUIRE(w.top <= static_cast<float>(env.clamp(static_cast<float>(s), 1e9f).elbow));
        REQUIRE(w.bottom >= static_cast<float>(env.clamp(static_cast<float>(s), -1e9f).elbow));
        REQUIRE(w.bottom <= w.top);
    }
}

// C++ port of mearm_pathb.py's ctrl_from_sensors: upper-arm gravity vector +
// elbow reading -> MeArm (base, shoulder, elbow) ctrl, via the spherical
// (tilt, azimuth) decode, pole/behind-body fades and the shoulder/elbow linkage
// projection. Python is the source of truth: data/pathb_golden.csv is generated
// from it (tools/mujoco_bridge/gen_pathb_golden.py) from the 2026-09-13 saved
// calibration and the 2026-09-24 REAL live log (each row paired with several
// elbow readings), and test_pathb_golden.py fails if the tables go stale.
//
// Why parity is pinned here (unlike the "assert properties" rule for real-
// hardware regressions): this is a cross-language port, and the point is that
// firmware == the Python that was validated in MuJoCo, exactly.
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_linkage.hpp"
#include "edgeneuro/control/mearm_pathb.hpp"

using Catch::Approx;
namespace P = edgeneuro::mearm::pathb;
namespace L = edgeneuro::mearm::linkage;

namespace {

std::vector<std::vector<float>> read_csv(const std::string& name) {
    std::ifstream f(std::string(EDGENEURO_DATA_DIR) + "/" + name);
    REQUIRE(f.is_open());
    std::string line;
    std::getline(f, line);  // header
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

P::Calibration golden_calibration() {
    const auto r = read_csv("pathb_calibration.csv").at(0);
    REQUIRE(r.size() == 13);
    P::Calibration cal;
    REQUIRE(P::Calibration::make({r[0], r[1], r[2]}, {r[3], r[4], r[5]}, {r[6], r[7], r[8]},
                                 {r[9], r[10], r[11]}, r[12], cal));
    return cal;
}

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

}  // namespace

TEST_CASE("C++ Path B reproduces every row of the Python golden table", "[mearm][pathb]") {
    const auto cal = golden_calibration();
    const auto rows = read_csv("pathb_golden.csv");
    REQUIRE(rows.size() >= 250);
    for (const auto& r : rows) {
        REQUIRE(r.size() == 7);
        const auto out = P::ctrl_from_sensors(cal, {r[0], r[1], r[2]}, r[3]);
        INFO("raw=(" << r[0] << "," << r[1] << "," << r[2] << ") bend=" << r[3]);
        REQUIRE(out.base == Approx(r[4]).margin(2e-3));
        REQUIRE(out.shoulder == Approx(r[5]).margin(2e-3));
        REQUIRE(out.elbow == Approx(r[6]).margin(2e-3));
    }
}

TEST_CASE("Path B output is always finite, inside the actuator ranges and inside the linkage band", "[mearm][pathb]") {
    const auto cal = golden_calibration();
    const float raws[][3] = {{0.99f, 0.05f, 0.27f}, {0.03f, -0.01f, 1.02f}, {0.12f, 0.45f, 0.92f},
                             {-1.0f, 0.0f, 0.0f},   {0.0f, 0.0f, -1.0f},    {0.0f, 0.0f, 0.0f},
                             {kNaN, 0.0f, 1.0f},    {kInf, 0.0f, 1.0f},     {1e-9f, 0.0f, 0.0f}};
    for (const auto& raw : raws) {
        for (float bend : {0.0f, 0.43f, 1.5f, 3.1f, -1.0f, kNaN, kInf, -kInf}) {
            const auto o = P::ctrl_from_sensors(cal, {raw[0], raw[1], raw[2]}, bend);
            INFO("raw=(" << raw[0] << "," << raw[1] << "," << raw[2] << ") bend=" << bend);
            REQUIRE(std::isfinite(o.base));
            REQUIRE(std::isfinite(o.shoulder));
            REQUIRE(std::isfinite(o.elbow));
            REQUIRE(std::fabs(o.base) <= P::kBaseLimit + 1e-4f);
            REQUIRE(o.shoulder >= L::kShoulderRaised - 1e-4f);
            REQUIRE(o.shoulder <= L::kShoulderRest + 1e-4f);
            REQUIRE(o.shoulder + o.elbow >= L::band_lo() - 1e-3f);
            REQUIRE(o.shoulder + o.elbow <= L::band_hi() + 1e-3f);
        }
    }
}

TEST_CASE("an invalid upper-arm reading commands the rest pose", "[mearm][pathb]") {
    const auto cal = golden_calibration();
    for (auto raw : {std::array<float, 3>{0.f, 0.f, 0.f}, std::array<float, 3>{kNaN, 0.f, 1.f},
                     std::array<float, 3>{kInf, 0.f, 1.f}}) {
        const auto o = P::ctrl_from_sensors(cal, raw, 0.43f);
        REQUIRE(o.base == 0.0f);
        REQUIRE(o.shoulder == Approx(L::kShoulderRest));
    }
}

TEST_CASE("the calibrated poses decode to the calibrated directions", "[mearm][pathb]") {
    const auto r = read_csv("pathb_calibration.csv").at(0);
    const auto cal = golden_calibration();
    const std::array<float, 3> hang{r[0], r[1], r[2]}, fwd{r[3], r[4], r[5]}, left{r[6], r[7], r[8]},
        right{r[9], r[10], r[11]};
    const float straight = r[12];
    REQUIRE(P::ctrl_from_sensors(cal, hang, straight).shoulder == Approx(L::kShoulderRest).margin(1e-3));
    REQUIRE(P::ctrl_from_sensors(cal, fwd, straight).shoulder == Approx(L::kShoulderRaised).margin(1e-3));
    REQUIRE(P::ctrl_from_sensors(cal, left, straight).base > 0.5f);     // arm left -> model left
    REQUIRE(P::ctrl_from_sensors(cal, right, straight).base < -0.5f);   // arm right -> model right
}

TEST_CASE("Calibration::make rejects an unusable calibration", "[mearm][pathb]") {
    const std::array<float, 3> hang{0.99f, 0.05f, 0.27f}, fwd{0.03f, -0.01f, 1.02f},
        left{0.12f, 0.45f, 0.92f}, right{0.11f, -0.46f, 0.90f}, zero{0.f, 0.f, 0.f};
    P::Calibration cal;
    REQUIRE(P::Calibration::make(hang, fwd, left, right, 0.43f, cal));
    REQUIRE_FALSE(P::Calibration::make(hang, hang, left, right, 0.43f, cal));    // FORWARD == HANG
    REQUIRE_FALSE(P::Calibration::make(hang, fwd, left, left, 0.43f, cal));      // LEFT/RIGHT same side
    // (swapped LEFT/RIGHT is NOT rejected -- and shouldn't be: +azimuth is defined by whichever
    //  vector is passed as LEFT, so it is just a mirrored calibration; Python behaves the same.)
    REQUIRE(P::Calibration::make(hang, fwd, right, left, 0.43f, cal));
    REQUIRE_FALSE(P::Calibration::make(zero, fwd, left, right, 0.43f, cal));     // zero vector
    REQUIRE_FALSE(P::Calibration::make(hang, fwd, left, right, kNaN, cal));      // NaN elbow zero
}

// C++ port of mearm_pathb.project_elbow (the shoulder/elbow linkage-coupling
// projection). Python is the source of truth: data/linkage_golden.csv is
// generated from mearm_pathb.py (tools/mujoco_bridge/gen_linkage_golden.py) and
// tools/mujoco_bridge/test_linkage_golden.py fails if it goes stale. This file
// checks (1) the port reproduces every golden row, (2) safety properties that
// hold for ANY input, independent of the golden values.
//
// Band limits are MeArmPilot's, not measured on this arm yet (SESSION_LOG TODO C).
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_linkage.hpp"

using Catch::Approx;
namespace L = edgeneuro::mearm::linkage;

TEST_CASE("C++ project_elbow reproduces every row of the Python golden table", "[mearm][linkage]") {
    std::ifstream f(std::string(EDGENEURO_DATA_DIR) + "/linkage_golden.csv");
    REQUIRE(f.is_open());
    std::string line;
    std::getline(f, line);  // header
    int rows = 0;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string a, b, c;
        std::getline(ss, a, ','); std::getline(ss, b, ','); std::getline(ss, c, ',');
        const float shoulder = std::stof(a), request = std::stof(b), expected = std::stof(c);
        INFO("shoulder=" << shoulder << " request=" << request);
        REQUIRE(L::project_elbow(shoulder, request) == Approx(expected).margin(1e-4));
        ++rows;
    }
    REQUIRE(rows >= 100);
}

TEST_CASE("projected shoulder+elbow always lies inside the linkage band", "[mearm][linkage]") {
    for (float s = L::kShoulderRaised; s <= L::kShoulderRest; s += 0.01f) {
        for (float r = -1.0f; r <= 4.0f; r += 0.05f) {
            const float e = L::project_elbow(s, r);
            REQUIRE(s + e >= L::band_lo() - 1e-4f);
            REQUIRE(s + e <= L::band_hi() + 1e-4f);
            REQUIRE(e >= L::kElbowExtended - 1e-4f);
            REQUIRE(e <= L::kElbowFolded + 1e-4f);
        }
    }
}

TEST_CASE("the elbow window is never empty at any shoulder height", "[mearm][linkage]") {
    for (float s = L::kShoulderRaised; s <= L::kShoulderRest; s += 0.01f) {
        REQUIRE(L::window_lo(s) < L::window_hi(s));
    }
}

TEST_CASE("projection is monotonic in the requested elbow", "[mearm][linkage]") {
    for (float s : {-0.14f, 0.2f, 0.5f, 0.9f}) {
        float prev = L::project_elbow(s, -1.0f);
        for (float r = -1.0f; r <= 4.0f; r += 0.02f) {
            const float e = L::project_elbow(s, r);
            REQUIRE(e >= prev - 1e-6f);
            prev = e;
        }
    }
}

TEST_CASE("NaN or infinite inputs still give a finite, in-window elbow", "[mearm][linkage]") {
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    constexpr float kInf = std::numeric_limits<float>::infinity();
    for (float s : {0.5f, kNaN, kInf, -kInf}) {
        for (float r : {1.5f, kNaN, kInf, -kInf}) {
            const float e = L::project_elbow(s, r);
            INFO("shoulder=" << s << " request=" << r);
            REQUIRE(std::isfinite(e));
            REQUIRE(e >= L::kElbowExtended - 1e-4f);
            REQUIRE(e <= L::kElbowFolded + 1e-4f);
        }
    }
}

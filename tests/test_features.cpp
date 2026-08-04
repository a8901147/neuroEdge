#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/features/mav_feature.hpp"
#include "edgeneuro/features/rms_feature.hpp"

using Catch::Approx;
using edgeneuro::MavFeature;
using edgeneuro::RmsFeature;

TEST_CASE("MavFeature averages absolute values", "[feature]") {
    MavFeature<float, 4> mav;
    const std::array<float, 4> window{-1.0f, 2.0f, -3.0f, 4.0f};
    REQUIRE(mav.compute(window) == Approx((1.0f + 2.0f + 3.0f + 4.0f) / 4.0f));
}

TEST_CASE("MavFeature of a constant window equals that constant's magnitude", "[feature]") {
    MavFeature<float, 100> mav;
    std::array<float, 100> window;
    window.fill(-2.5f);
    REQUIRE(mav.compute(window) == Approx(2.5f));
}

TEST_CASE("RmsFeature computes root-mean-square", "[feature]") {
    RmsFeature<float, 4> rms;
    const std::array<float, 4> window{3.0f, 3.0f, 3.0f, 3.0f};
    REQUIRE(rms.compute(window) == Approx(3.0f));
}

TEST_CASE("RmsFeature of mixed-sign window matches hand-computed value", "[feature]") {
    RmsFeature<float, 4> rms;
    const std::array<float, 4> window{1.0f, -1.0f, 2.0f, -2.0f};
    // sqrt((1+1+4+4)/4) = sqrt(2.5)
    REQUIRE(rms.compute(window) == Approx(1.5811388f).epsilon(0.0001));
}

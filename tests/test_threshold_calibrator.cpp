#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/threshold_calibrator.hpp"

using Catch::Approx;
using edgeneuro::ThresholdCalibrator;

TEST_CASE("ThresholdCalibrator is invalid before any observations", "[control]") {
    ThresholdCalibrator<float> cal;
    REQUIRE_FALSE(cal.is_valid());
}

TEST_CASE("ThresholdCalibrator computes the midpoint between relaxed max and contracted min", "[control]") {
    ThresholdCalibrator<float> cal;
    for (float v : {434.0f, 480.0f, 495.0f, 460.0f}) {
        cal.observe_relaxed(v);
    }
    for (float v : {3689.0f, 3800.0f, 3700.0f}) {
        cal.observe_contracted(v);
    }
    REQUIRE(cal.is_valid());
    // relaxed_max=495, contracted_min=3689 -> midpoint=2092
    REQUIRE(cal.threshold() == Approx((495.0f + 3689.0f) / 2.0f));
}

TEST_CASE("ThresholdCalibrator matches this project's real Stage 5a data", "[control]") {
    // PRD.md Stage 5a: relaxed ~434-495, sustained clench ~3700+.
    ThresholdCalibrator<float> cal;
    for (float v : {434.0f, 495.0f, 484.0f, 478.0f, 479.0f}) {
        cal.observe_relaxed(v);
    }
    for (float v : {3700.0f, 3731.0f, 3689.0f, 3717.0f}) {
        cal.observe_contracted(v);
    }
    REQUIRE(cal.is_valid());
    const float t = cal.threshold();
    REQUIRE(t > 495.0f);  // clears the relaxed ceiling
    REQUIRE(t < 3689.0f); // clears under the contracted floor
}

TEST_CASE("ThresholdCalibrator is invalid when relaxed and contracted ranges overlap", "[control]") {
    // Simulates a bad calibration run -- e.g. poor electrode contact, or
    // the user didn't actually contract during the "contract" phase.
    ThresholdCalibrator<float> cal;
    cal.observe_relaxed(1000.0f);
    cal.observe_contracted(800.0f); // lower than the relaxed reading -- no real separation
    REQUIRE_FALSE(cal.is_valid());
}

TEST_CASE("ThresholdCalibrator only keeps the loudest relaxed sample and quietest contracted sample", "[control]") {
    ThresholdCalibrator<float> cal;
    cal.observe_relaxed(400.0f);
    cal.observe_relaxed(500.0f); // louder -- should win
    cal.observe_relaxed(450.0f);
    cal.observe_contracted(4000.0f);
    cal.observe_contracted(3600.0f); // quieter -- should win
    cal.observe_contracted(3900.0f);

    REQUIRE(cal.threshold() == Approx((500.0f + 3600.0f) / 2.0f));
}

TEST_CASE("ThresholdCalibrator exposes relaxed_max/contracted_min for diagnostics", "[control]") {
    ThresholdCalibrator<float> cal;
    cal.observe_relaxed(400.0f);
    cal.observe_relaxed(480.0f);
    cal.observe_contracted(3600.0f);
    cal.observe_contracted(3900.0f);
    REQUIRE(cal.relaxed_max() == Approx(480.0f));
    REQUIRE(cal.contracted_min() == Approx(3600.0f));
}

TEST_CASE("ThresholdCalibrator::reset clears prior observations", "[control]") {
    ThresholdCalibrator<float> cal;
    cal.observe_relaxed(500.0f);
    cal.observe_contracted(3700.0f);
    REQUIRE(cal.is_valid());

    cal.reset();
    REQUIRE_FALSE(cal.is_valid());

    // Confirms relaxed_max_ was actually reset to 0, not left at 500 --
    // otherwise a single low contracted-looking sample right after reset
    // would spuriously validate.
    cal.observe_contracted(10.0f);
    REQUIRE_FALSE(cal.is_valid());
}

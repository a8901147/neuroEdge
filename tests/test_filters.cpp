#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/filters/pass_through_filter.hpp"

using Catch::Approx;
using edgeneuro::IirFilter;
using edgeneuro::PassThroughFilter;

TEST_CASE("PassThroughFilter is the identity function", "[filter]") {
    PassThroughFilter<float> filter;
    REQUIRE(filter.process(3.14f) == Approx(3.14f));
    REQUIRE(filter.process(-2.0f) == Approx(-2.0f));
    filter.reset(); // must not throw, must not change subsequent behavior
    REQUIRE(filter.process(0.0f) == Approx(0.0f));
}

TEST_CASE("IirFilter identity coefficients pass the signal through unchanged", "[filter]") {
    // b0=1, everything else 0 => output == input every sample.
    IirFilter<float> filter(1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    REQUIRE(filter.process(1.0f) == Approx(1.0f));
    REQUIRE(filter.process(-5.0f) == Approx(-5.0f));
    REQUIRE(filter.process(0.0f) == Approx(0.0f));
}

TEST_CASE("IirFilter reset clears delay-line state", "[filter]") {
    // A filter with feedback (a1 != 0) is state-dependent; after reset it
    // must reproduce the exact same first-sample output as a fresh filter.
    IirFilter<float> filter(0.5f, 0.5f, 0.0f, -0.5f, 0.0f);
    const float first_response = filter.process(1.0f);
    filter.process(1.0f);
    filter.process(1.0f);
    filter.reset();
    REQUIRE(filter.process(1.0f) == Approx(first_response));
}

TEST_CASE("IirFilter step response converges for a stable low-pass design", "[filter]") {
    // One-pole low-pass: y[n] = (1-a)*x[n] + a*y[n-1], written in Direct
    // Form II as b0=(1-a), a1=-a. A unit step should converge to gain 1.
    const float a = 0.9f;
    IirFilter<float> filter(1.0f - a, 0.0f, 0.0f, -a, 0.0f);
    float output = 0.0f;
    for (int i = 0; i < 500; ++i) {
        output = filter.process(1.0f);
    }
    REQUIRE(output == Approx(1.0f).epsilon(0.001));
}

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/grip_state_machine.hpp"

using edgeneuro::GripStateMachine;

TEST_CASE("GripStateMachine starts Released", "[control]") {
    GripStateMachine<float> gsm(/*threshold=*/100.0f, /*on_duration=*/0.1f, /*off_duration=*/0.1f);
    REQUIRE(gsm.state() == GripStateMachine<float>::State::Released);
    REQUIRE_FALSE(gsm.is_gripping());
}

// Note: durations are checked well short of / well past the exact boundary
// (not at, say, exactly the 10th 10ms tick for a 0.1s duration) -- summing
// 0.01f ten times lands a hair below 0.1f due to float rounding, so pinning
// down the *exact* tick that crosses the threshold is not a meaningful
// thing to assert. What matters is that short holds don't fire and long
// holds do.

TEST_CASE("GripStateMachine does not transition before on_duration is reached", "[control]") {
    GripStateMachine<float> gsm(100.0f, /*on_duration=*/0.1f, /*off_duration=*/0.1f);
    for (int i = 0; i < 8; ++i) { // 0.08s -- comfortably short of 0.1s
        const bool edge = gsm.update(200.0f, 0.01f);
        REQUIRE_FALSE(edge);
        REQUIRE_FALSE(gsm.is_gripping());
    }
}

TEST_CASE("GripStateMachine transitions to Gripping once on_duration is exceeded", "[control]") {
    GripStateMachine<float> gsm(100.0f, /*on_duration=*/0.1f, /*off_duration=*/0.1f);
    bool ever_fired = false;
    for (int i = 0; i < 12; ++i) { // 0.12s -- comfortably past 0.1s
        if (gsm.update(200.0f, 0.01f)) {
            REQUIRE_FALSE(ever_fired); // fires exactly once, not repeatedly
            ever_fired = true;
        }
    }
    REQUIRE(ever_fired);
    REQUIRE(gsm.is_gripping());
}

TEST_CASE("GripStateMachine rejects a brief above-threshold blip shorter than on_duration", "[control]") {
    // A noise spike that doesn't hold long enough must not trigger a grip --
    // this is the entire point of the duration requirement, not just the
    // threshold comparison.
    GripStateMachine<float> gsm(100.0f, /*on_duration=*/0.1f, /*off_duration=*/0.1f);

    for (int i = 0; i < 5; ++i) {
        gsm.update(200.0f, 0.01f); // 0.05s above threshold -- short of 0.1s
    }
    REQUIRE_FALSE(gsm.is_gripping());

    const bool edge = gsm.update(0.0f, 0.01f); // drops back below threshold
    REQUIRE_FALSE(edge);
    REQUIRE_FALSE(gsm.is_gripping());
}

TEST_CASE("GripStateMachine transitions back to Released once off_duration is exceeded", "[control]") {
    GripStateMachine<float> gsm(100.0f, /*on_duration=*/0.05f, /*off_duration=*/0.1f);

    for (int i = 0; i < 8; ++i) { // comfortably past on_duration (0.05s)
        gsm.update(200.0f, 0.01f);
    }
    REQUIRE(gsm.is_gripping());

    for (int i = 0; i < 8; ++i) { // 0.08s below threshold -- comfortably short of 0.1s
        const bool edge = gsm.update(0.0f, 0.01f);
        REQUIRE_FALSE(edge);
        REQUIRE(gsm.is_gripping());
    }

    bool ever_fired = false;
    for (int i = 0; i < 5; ++i) { // pushes total below-threshold time comfortably past 0.1s
        if (gsm.update(0.0f, 0.01f)) {
            REQUIRE_FALSE(ever_fired);
            ever_fired = true;
        }
    }
    REQUIRE(ever_fired);
    REQUIRE_FALSE(gsm.is_gripping());
}

TEST_CASE("GripStateMachine ignores a brief below-threshold dip while Gripping", "[control]") {
    GripStateMachine<float> gsm(100.0f, /*on_duration=*/0.05f, /*off_duration=*/0.1f);
    for (int i = 0; i < 8; ++i) {
        gsm.update(200.0f, 0.01f);
    }
    REQUIRE(gsm.is_gripping());

    for (int i = 0; i < 3; ++i) {
        gsm.update(0.0f, 0.01f); // 0.03s below threshold -- short of 0.1s
    }
    REQUIRE(gsm.is_gripping());

    gsm.update(200.0f, 0.01f); // back above threshold before off_duration elapsed
    REQUIRE(gsm.is_gripping());
}

TEST_CASE("GripStateMachine::reset clears accumulated time and returns to Released", "[control]") {
    GripStateMachine<float> gsm(100.0f, 0.05f, 0.1f);
    for (int i = 0; i < 8; ++i) {
        gsm.update(200.0f, 0.01f);
    }
    REQUIRE(gsm.is_gripping());

    gsm.reset();
    REQUIRE_FALSE(gsm.is_gripping());
    REQUIRE(gsm.state() == GripStateMachine<float>::State::Released);

    // Confirms accumulated above_time_ was actually cleared, not just the
    // state flag -- if it weren't, a single subsequent sample would
    // immediately re-trigger Gripping.
    const bool edge = gsm.update(200.0f, 0.01f);
    REQUIRE_FALSE(edge);
    REQUIRE_FALSE(gsm.is_gripping());
}

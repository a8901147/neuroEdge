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

TEST_CASE("GripStateMachine::set_threshold changes future comparisons without resetting state", "[control]") {
    GripStateMachine<float> gsm(100.0f, /*on_duration=*/0.05f, /*off_duration=*/0.1f);
    for (int i = 0; i < 8; ++i) {
        gsm.update(150.0f, 0.01f); // above the original threshold (100)
    }
    REQUIRE(gsm.is_gripping());

    gsm.set_threshold(200.0f); // 150 is now BELOW the new threshold
    REQUIRE(gsm.threshold() == 200.0f);
    // Still gripping immediately after the change -- set_threshold alone
    // doesn't force a re-evaluation or reset accumulated time.
    REQUIRE(gsm.is_gripping());

    // But now that 150 no longer counts as "above", holding it long enough
    // releases via the normal off_duration path, proving the NEW threshold
    // is what update() compares against, not the constructor's original one.
    bool ever_released = false;
    for (int i = 0; i < 12; ++i) {
        if (gsm.update(150.0f, 0.01f)) {
            ever_released = true;
        }
    }
    REQUIRE(ever_released);
    REQUIRE_FALSE(gsm.is_gripping());
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

// 2026-10-04: two thresholds (hysteresis). The user found the grip "lets go too easily": in the 2026-10-04 13:12
// calibration the relaxed level was ~1170, the threshold 1876, and the clench's lowest 10% only ~1908 (5% of it already
// below) -- a grip held more gently while the arm moves dips under 1876 and is released after 0.15 s. Gripping still
// needs the full threshold; once gripping, only falling below the lower RELEASE threshold for off_duration lets go.
TEST_CASE("GripStateMachine with a release threshold: a gentler hold between the two thresholds keeps the grip",
          "[control][grip_hysteresis]") {
    edgeneuro::GripStateMachine<float> g(1876.0f, 0.15f, 0.15f);
    g.set_thresholds(1876.0f, 1523.0f);
    for (int i = 0; i < 200; ++i) g.update(2300.0f, 0.001f);      // a firm clench
    REQUIRE(g.is_gripping());
    for (int i = 0; i < 2000; ++i) g.update(1700.0f, 0.001f);     // 2 s held more gently: below 1876, above 1523
    REQUIRE(g.is_gripping());
}

TEST_CASE("GripStateMachine with a release threshold: relaxing below it still releases after off_duration",
          "[control][grip_hysteresis]") {
    edgeneuro::GripStateMachine<float> g(1876.0f, 0.15f, 0.15f);
    g.set_thresholds(1876.0f, 1523.0f);
    for (int i = 0; i < 200; ++i) g.update(2300.0f, 0.001f);
    for (int i = 0; i < 140; ++i) g.update(1170.0f, 0.001f);      // relaxed, but not yet for 0.15 s
    REQUIRE(g.is_gripping());
    bool released = false;
    for (int i = 0; i < 20; ++i) released = g.update(1170.0f, 0.001f) || released;
    REQUIRE(released);
    REQUIRE_FALSE(g.is_gripping());
}

TEST_CASE("GripStateMachine with a release threshold: gripping still needs the full threshold",
          "[control][grip_hysteresis]") {
    edgeneuro::GripStateMachine<float> g(1876.0f, 0.15f, 0.15f);
    g.set_thresholds(1876.0f, 1523.0f);
    for (int i = 0; i < 2000; ++i) g.update(1700.0f, 0.001f);     // above release, below grip: not a grip
    REQUIRE_FALSE(g.is_gripping());
}

TEST_CASE("GripStateMachine: set_threshold alone means one threshold for both, exactly as before",
          "[control][grip_hysteresis]") {
    edgeneuro::GripStateMachine<float> g(1000.0f, 0.15f, 0.15f);
    g.set_thresholds(1876.0f, 1523.0f);
    g.set_threshold(1876.0f);
    REQUIRE(g.threshold() == 1876.0f);
    REQUIRE(g.release_threshold() == 1876.0f);
    for (int i = 0; i < 200; ++i) g.update(2300.0f, 0.001f);
    for (int i = 0; i < 200; ++i) g.update(1700.0f, 0.001f);      // below the single threshold: released
    REQUIRE_FALSE(g.is_gripping());
}

TEST_CASE("GripStateMachine: a release threshold above the grip threshold is capped at it",
          "[control][grip_hysteresis]") {
    edgeneuro::GripStateMachine<float> g(1000.0f, 0.15f, 0.15f);
    g.set_thresholds(1500.0f, 1800.0f);
    REQUIRE(g.release_threshold() == 1500.0f);
}

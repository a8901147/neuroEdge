// The EMG threshold line run_demo_live.py sends, with an optional release threshold for the two-threshold grip (see
// test_grip_state_machine.cpp [grip_hysteresis]). Parsed one byte at a time by the firmware's RX path.
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/emg_threshold_link.hpp"

using edgeneuro::emg_threshold_link::Parser;

namespace {
bool feed_all(Parser& p, const char* s) {
    bool done = false;
    for (; *s; ++s) done = p.feed(static_cast<uint8_t>(*s)) || done;
    return done;
}
}  // namespace

TEST_CASE("threshold line: T<on> alone means the same release threshold (the old format)", "[emg_threshold_link]") {
    Parser p;
    REQUIRE(feed_all(p, "T1876\n"));
    REQUIRE(p.on() == 1876u);
    REQUIRE(p.release() == 1876u);
}

TEST_CASE("threshold line: T<on>,<release>", "[emg_threshold_link]") {
    Parser p;
    REQUIRE(feed_all(p, "T1876,1523\r\n"));
    REQUIRE(p.on() == 1876u);
    REQUIRE(p.release() == 1523u);
}

TEST_CASE("threshold line: only a complete line counts; a new T restarts", "[emg_threshold_link]") {
    Parser p;
    REQUIRE_FALSE(feed_all(p, "T1876,15"));
    REQUIRE(feed_all(p, "T2000\n"));
    REQUIRE(p.on() == 2000u);
    REQUIRE(p.release() == 2000u);
}

TEST_CASE("threshold line: bytes outside a T line are ignored (other commands share the port)", "[emg_threshold_link]") {
    Parser p;
    REQUIRE_FALSE(feed_all(p, "R\nC12,34\n"));
}

TEST_CASE("threshold line: garbage is refused, not half-applied", "[emg_threshold_link]") {
    Parser p;
    REQUIRE_FALSE(feed_all(p, "T\n"));                 // no value
    REQUIRE_FALSE(feed_all(p, "T1876,\n"));            // empty release
    REQUIRE_FALSE(feed_all(p, "T18x76\n"));            // a stray character
    REQUIRE_FALSE(feed_all(p, "T1,2,3\n"));            // three values
    REQUIRE_FALSE(feed_all(p, "T12345678\n"));         // far beyond a 12-bit ADC: garbage
}

TEST_CASE("threshold line: values that would grip at rest, never grip or never release are refused",
          "[emg_threshold_link]") {
    Parser p;
    REQUIRE_FALSE(feed_all(p, "T0\n"));                // 0: every sample is above it, the claw closes at rest
    REQUIRE_FALSE(feed_all(p, "T4096\n"));             // above the 12-bit ADC: can never be reached
    REQUIRE_FALSE(feed_all(p, "T1876,0\n"));           // release 0: never lets go
    REQUIRE_FALSE(feed_all(p, "T1500,1600\n"));        // release above the grip threshold
    REQUIRE(feed_all(p, "T4095,1\n"));                 // the extremes of a valid pair
    REQUIRE(p.on() == 4095u);
    REQUIRE(p.release() == 1u);
}

TEST_CASE("threshold line: a refused line keeps the thresholds already received", "[emg_threshold_link]") {
    Parser p;
    REQUIRE(feed_all(p, "T1876,1523\n"));
    REQUIRE_FALSE(feed_all(p, "T0\n"));
    REQUIRE(p.on() == 1876u);
    REQUIRE(p.release() == 1523u);
}

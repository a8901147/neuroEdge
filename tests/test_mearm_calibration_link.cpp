// The MEArm calibration sent over UART (2026-10-03, option B): instead of compiling the calibration into the firmware
// and re-flashing after every re-calibration, run_demo_live.py sends it as
//   C<v0>,<v1>,...,<v19>,<checksum>\n
// 20 values, each round(value * 1e6) as a signed decimal integer: HANG xyz, FORWARD xyz, LEFT xyz, RIGHT xyz,
// zero_elbow, has_reach (0/1), base reach LEFT xyz, base reach RIGHT xyz; checksum = the sum of the 20 integers mod
// 1000000007. The parser takes one byte at a time (the firmware's RX path), allocates nothing, and accepts a message only
// if it is complete and its checksum matches. Python encoder: tools/mujoco_bridge/mearm_calibration_link.py.
#include <cmath>
#include <cstdint>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mearm_calibration_link.hpp"

namespace L = edgeneuro::mearm::calibration_link;

namespace {
// feed a whole string; returns how many complete, valid messages came out
int feed(L::Parser& p, const std::string& s) {
    int done = 0;
    for (char c : s) done += p.feed(static_cast<uint8_t>(c)) ? 1 : 0;
    return done;
}
std::string message(const long long (&v)[L::kFieldCount], long long checksum_offset = 0) {
    std::string s = "C";
    long long sum = 0;
    for (int i = 0; i < L::kFieldCount; ++i) {
        s += std::to_string(v[i]) + ",";
        sum += v[i];
    }
    const long long m = 1000000007LL;
    s += std::to_string((((sum % m) + m) % m + checksum_offset) % m) + "\n";
    return s;
}
// the 9/13 calibration, x1e6 (rounded), no measured reach
constexpr long long k913[L::kFieldCount] = {992637, 46787, 265020, 33409, -10960, 1024878, 122861, 451801, 918384,
                                            112252, -461119, 899955, 434012, 0, 0, 0, 0, 0, 0, 0};
}  // namespace

TEST_CASE("a complete message with the right checksum is accepted and decoded", "[mearm][calibration_link]") {
    L::Parser p;
    REQUIRE(feed(p, message(k913)) == 1);
    const L::Values& v = p.values();
    REQUIRE(v.hang[0] == 0.992637f);
    REQUIRE(v.forward[2] == 1.024878f);
    REQUIRE(v.right[1] == -0.461119f);
    REQUIRE(v.zero_elbow == 0.434012f);
    REQUIRE_FALSE(v.has_reach);
}

TEST_CASE("a measured base reach is decoded too", "[mearm][calibration_link]") {
    long long v[L::kFieldCount];
    for (int i = 0; i < L::kFieldCount; ++i) v[i] = k913[i];
    v[13] = 1000000;
    v[14] = 300000; v[15] = 600000; v[16] = 750000;
    v[17] = 250000; v[18] = -620000; v[19] = 740000;
    L::Parser p;
    REQUIRE(feed(p, message(v)) == 1);
    REQUIRE(p.values().has_reach);
    REQUIRE(p.values().reach_left[1] == 0.6f);
    REQUIRE(p.values().reach_right[1] == -0.62f);
}

TEST_CASE("a wrong checksum, a missing field or an extra field is rejected and counted", "[mearm][calibration_link]") {
    L::Parser p;
    REQUIRE(feed(p, message(k913, 1)) == 0);                       // one off
    std::string short_msg = message(k913);
    short_msg.erase(short_msg.find(','), short_msg.find(',', short_msg.find(',') + 1) - short_msg.find(','));
    REQUIRE(feed(p, short_msg) == 0);                              // a field dropped
    std::string long_msg = message(k913);
    long_msg.insert(1, "5,");
    REQUIRE(feed(p, long_msg) == 0);                               // an extra field
    REQUIRE(p.rejected() == 3u);
    REQUIRE(feed(p, message(k913)) == 1);                          // and a good one still works afterwards
}

TEST_CASE("garbage inside a message rejects it; bytes outside a message are ignored", "[mearm][calibration_link]") {
    L::Parser p;
    std::string bad = message(k913);
    bad[5] = 'x';
    REQUIRE(feed(p, bad) == 0);
    REQUIRE(p.rejected() == 1u);
    REQUIRE(feed(p, "T1129\nR\n") == 0);                           // other commands: not ours, not counted
    REQUIRE(p.rejected() == 1u);
}

TEST_CASE("a new C restarts an unfinished message (a dropped newline cannot glue two together)", "[mearm][calibration_link]") {
    L::Parser p;
    std::string half = message(k913).substr(0, 40);
    REQUIRE(feed(p, half + message(k913)) == 1);
}

TEST_CASE("an overlong number is rejected instead of overflowing", "[mearm][calibration_link]") {
    L::Parser p;
    std::string s = message(k913);
    s.insert(1, "99999999999999999999");
    REQUIRE(feed(p, s) == 0);
    REQUIRE(p.rejected() == 1u);
}

TEST_CASE("a stray byte between digits rejects the message rather than being skipped", "[mearm][calibration_link]") {
    // skipping it would silently splice two numbers' digits -- the checksum might not notice a corrupted SPACE
    L::Parser p;
    std::string s = message(k913);
    s.insert(s.find(',') + 1, " ");
    REQUIRE(feed(p, s) == 0);
    REQUIRE(p.rejected() == 1u);
}

TEST_CASE("more than 12 digits in one number is rejected even when the value itself would be fine", "[mearm][calibration_link]") {
    L::Parser p;
    std::string s = message(k913);
    s.insert(1, "0000000");                                        // 992637 -> 0000000992637: 13 digits, same value
    REQUIRE(feed(p, s) == 0);
    REQUIRE(p.rejected() == 1u);
}

// ---- using a received calibration (mearm::apply_calibration_message): validated like the compiled-in one ----
#include "edgeneuro/control/mearm_drive.hpp"

namespace {
L::Values values_913() {
    L::Parser p;
    REQUIRE(feed(p, message(k913)) == 1);
    return p.values();
}
}  // namespace

TEST_CASE("a received 9/13 calibration is used, and equals the compiled-in 9/13 one", "[mearm][calibration_link]") {
    edgeneuro::mearm::pathb::Calibration cal, compiled;
    edgeneuro::mearm::real::BaseReach reach{};
    REQUIRE(edgeneuro::mearm::apply_calibration_message(values_913(), cal, reach));
    REQUIRE(edgeneuro::mearm::make_compiled_calibration(compiled));
    REQUIRE(std::fabs(cal.az_left - compiled.az_left) < 1e-5f);
    REQUIRE(std::fabs(cal.tilt_forward - compiled.tilt_forward) < 1e-5f);
    const auto def = edgeneuro::mearm::real::default_base_reach(cal);
    REQUIRE((reach.left == def.left && reach.right == def.right));     // no measured reach -> the default
}

TEST_CASE("a received calibration Path B rejects is NOT used: the current one stays", "[mearm][calibration_link]") {
    edgeneuro::mearm::pathb::Calibration cal;
    REQUIRE(edgeneuro::mearm::make_compiled_calibration(cal));
    edgeneuro::mearm::real::BaseReach reach{0.7f, -0.7f};
    const float before = cal.az_left;
    L::Values bad = values_913();
    bad.forward = bad.hang;                                             // FORWARD == HANG: unusable
    REQUIRE_FALSE(edgeneuro::mearm::apply_calibration_message(bad, cal, reach));
    REQUIRE(cal.az_left == before);
    REQUIRE((reach.left == 0.7f && reach.right == -0.7f));
}

TEST_CASE("a received reach on one side only is NOT used either", "[mearm][calibration_link]") {
    edgeneuro::mearm::pathb::Calibration cal;
    REQUIRE(edgeneuro::mearm::make_compiled_calibration(cal));
    edgeneuro::mearm::real::BaseReach reach{0.7f, -0.7f};
    L::Values v = values_913();
    v.has_reach = true;
    v.reach_left = v.left;
    v.reach_right = v.left;                                             // both on the left
    REQUIRE_FALSE(edgeneuro::mearm::apply_calibration_message(v, cal, reach));
    REQUIRE((reach.left == 0.7f && reach.right == -0.7f));
}

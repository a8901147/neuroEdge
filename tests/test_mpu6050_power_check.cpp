// edgeneuro::mpu6050::power_check: reading the sensor's OWN power register instead of inferring its state from the data
// (2026-09-28, the author's request: use what the hardware says, not the result). Register facts are independent literals
// from InvenSense RM-MPU-6000A-00 Rev 4.0 (checked against the original text, SESSION_LOG 2026-09-28):
//   PWR_MGMT_1 = register 107 (0x6B); bit 6 = SLEEP; reset value 0x40 (all other registers except WHO_AM_I reset to 0x00);
//   USER_CTRL 0x6A, PWR_MGMT_2 0x6C are documented neighbours with no read side effect (0x6D-0x71 are undocumented and
//   0x74 FIFO_R_W pops the FIFO, so the check reads exactly 0x6A..0x6C).
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/mpu6050_power_check.hpp"

namespace P = edgeneuro::mpu6050;

TEST_CASE("the health read covers exactly the documented, side-effect-free registers 0x6A..0x6C", "[mpu6050]") {
    REQUIRE(P::kHealthReadStartReg == 0x6Au);
    REQUIRE(P::kHealthReadLen == 3u);
    REQUIRE(P::kPwrMgmt1Index == 1u);                     // 0x6B is the middle byte
    REQUIRE(P::kHealthReadStartReg + P::kPwrMgmt1Index == 0x6Bu);
    REQUIRE(P::kHealthReadLen >= 3u);                     // the firmware's verified multi-byte read needs >= 3
}

TEST_CASE("the value the firmware writes at wake-up reads back as awake", "[mpu6050]") {
    REQUIRE(P::classify_pwr_mgmt_1(0x01u) == P::PowerState::Awake);   // SLEEP=0, CLKSEL=1 (X gyro PLL)
}

TEST_CASE("the power-on reset value 0x40 (SLEEP=1) means the sensor reset and is asleep", "[mpu6050]") {
    REQUIRE(P::classify_pwr_mgmt_1(0x40u) == P::PowerState::ResetAsleep);
    REQUIRE(P::classify_pwr_mgmt_1(0x41u) == P::PowerState::ResetAsleep);   // any value with SLEEP set
}

TEST_CASE("an awake but unexpected configuration is reported as such, not as healthy", "[mpu6050]") {
    REQUIRE(P::classify_pwr_mgmt_1(0x00u) == P::PowerState::Unexpected);    // awake on the internal oscillator
    REQUIRE(P::classify_pwr_mgmt_1(0x81u) == P::PowerState::Unexpected);    // DEVICE_RESET bit set
    REQUIRE(P::classify_pwr_mgmt_1(0xFFu) == P::PowerState::ResetAsleep);   // SLEEP set wins: it needs waking either way
}

TEST_CASE("a health read is due once per period per sensor, starting one period after boot", "[mpu6050]") {
    P::HealthSchedule s(1000u);
    REQUIRE_FALSE(s.due(0u));
    REQUIRE_FALSE(s.due(999u));
    REQUIRE(s.due(1000u));
    s.mark_done(1000u);
    REQUIRE_FALSE(s.due(1500u));
    REQUIRE(s.due(2000u));
}

TEST_CASE("the schedule survives the 32-bit tick counter wrapping", "[mpu6050]") {
    P::HealthSchedule s(1000u);
    s.mark_done(0xFFFFFF00u);
    REQUIRE_FALSE(s.due(0xFFFFFFFFu));
    REQUIRE(s.due(0x000002F0u));                           // 0x100 + 0x2F0 = 1008 ticks later
}

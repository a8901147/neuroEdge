#pragma once

#include <cstdint>

namespace edgeneuro::mpu6050 {

// Reading the MPU6050's OWN power register, instead of inferring its state from the data it returns (2026-09-28).
//
// Register facts, checked against the original text of InvenSense "MPU-6000/MPU-6050 Register Map and Descriptions",
// RM-MPU-6000A-00 Rev 4.0 (SESSION_LOG 2026-09-28):
//   * PWR_MGMT_1 is register 107 (0x6B): bit7 DEVICE_RESET, bit6 SLEEP, bit5 CYCLE, bit3 TEMP_DIS, bits2:0 CLKSEL;
//   * its reset value is 0x40 (SLEEP=1); every other register except WHO_AM_I resets to 0x00;
//   * USER_CTRL (0x6A) and PWR_MGMT_2 (0x6C) are documented and have no read side effect. 0x6D-0x71 are NOT documented
//     and 0x74 (FIFO_R_W) pops the FIFO when read, so the health read is exactly 0x6A..0x6C: 3 bytes, which also satisfies
//     the firmware's verified multi-byte I2C read sequence (it needs at least 3 bytes).
// The firmware writes PWR_MGMT_1 = 0x01 at wake-up (SLEEP=0, CLKSEL=1: PLL with X-gyro reference). A sensor that lost power
// for a moment between two reads comes back reset -- SLEEP=1 -- without any I2C error the firmware could have seen.
constexpr uint8_t kHealthReadStartReg = 0x6Au;
constexpr uint32_t kHealthReadLen = 3u;
constexpr uint32_t kPwrMgmt1Index = 1u;          // 0x6B within the 0x6A..0x6C read
constexpr uint8_t kPwrMgmt1Awake = 0x01u;        // what the firmware writes at wake-up
constexpr uint8_t kSleepBit = 0x40u;

enum class PowerState : uint8_t {
    Awake,          // reads back exactly what the firmware wrote
    ResetAsleep,    // SLEEP set: the sensor reset (power blip) -- its data registers stop updating; needs waking again
    Unexpected,     // awake, but not the configuration the firmware wrote -- report it, do not call it healthy
};

constexpr PowerState classify_pwr_mgmt_1(uint8_t value) noexcept {
    if ((value & kSleepBit) != 0u) return PowerState::ResetAsleep;
    if (value == kPwrMgmt1Awake) return PowerState::Awake;
    return PowerState::Unexpected;
}

// When a sensor's next read should be the health read instead of the data read: once per `period_ticks`
// (1 kHz ticks), starting one period after boot. Wrap-safe (unsigned difference).
class HealthSchedule {
public:
    constexpr explicit HealthSchedule(uint32_t period_ticks) noexcept : period_(period_ticks) {}
    constexpr bool due(uint32_t tick) const noexcept { return tick - last_ >= period_; }
    constexpr void mark_done(uint32_t tick) noexcept { last_ = tick; }

private:
    uint32_t period_;
    uint32_t last_ = 0u;
};

}  // namespace edgeneuro::mpu6050

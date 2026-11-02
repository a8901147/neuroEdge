// Two MPU6050s on I2C1 (PB6 = SCL, PB7 = SDA, 100 kHz): blocking register writes for setup, bus-clear recovery, and a
// non-blocking read state machine for the main loop.
// Included only by phase3_control_loop_main.cpp (one translation unit).
#pragma once

#include <cstdint>

#include "board.hpp"
#include "stm32f4xx.h"

static constexpr uint8_t kShoulderImuAddr = 0x68u; // upper-arm MPU6050, AD0 low
static constexpr uint8_t kElbowImuAddr = 0x69u;    // forearm MPU6050, AD0 tied to 3.3 V
static constexpr uint8_t kImuRegAddr = 0x3Bu;      // ACCEL_XOUT_H: accel, temperature, gyro follow
static constexpr uint32_t kImuReadLen = 14u;
static constexpr uint32_t kImuMaxTicksPerRead = 50u; // abort and recover a read stuck for more than 50 ms

// CONFIG (0x1A) DLPF_CFG = 6: 5 Hz accel/gyro bandwidth, ~19 ms delay (RM-MPU-6000A-00 4.3). Chosen because the arm
// shook during a sustained grip: gyro spread was ~100x the resting noise at 9-10 Hz, i.e. physiological tremor, while
// voluntary arm motion stays below ~3 Hz. With CFG = 6 the spread during a firm grip fell from 0.5 to 0.02 rad/s
// (SESSION_LOG 2026-09-12). The reset default (0) is effectively unfiltered.
static constexpr uint8_t kMpu6050ConfigReg = 0x1Au;
static constexpr uint8_t kDlpfCfg6 = 0x06u;

// I2C1 at 100 kHz from the 16 MHz PCLK1 (RM0368 18.6.8/18.6.9). 400 kHz was tried: about 3.4x more reads per second,
// but it reproducibly hung the bus on the breadboard wiring (SESSION_LOG, August 2026), so the bus stays at 100 kHz.
static constexpr uint32_t kI2cCcr100k = 0x50u;
static constexpr uint32_t kI2cTrise100k = 0x11u;

// Readable over SWD (openocd mdw) when the bus misbehaves.
volatile int g_imu_state_at_timeout = -1;
volatile uint32_t g_sr1_at_timeout = 0xFFFFFFFFu;
volatile uint32_t g_sr2_at_timeout = 0xFFFFFFFFu;
volatile uint32_t g_timeout_count = 0;
volatile int g_wake_result_shoulder = -1;
volatile int g_wake_result_elbow = -1;

// Blocking single-register write, bounded by guard counters. Used at boot and to re-wake a sensor; a few hundred us at
// 100 kHz, rare enough to stay blocking. Returns 0 on success, otherwise the step that failed (1..4).
static int mpu6050_write_reg_blocking(uint8_t addr7, uint8_t reg, uint8_t value) {
    uint32_t guard = 100000u;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--guard == 0) return 1;
    }

    I2C1->DR = (uint8_t)(addr7 << 1);
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 2;
        }
        if (--guard == 0) return 2;
    }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (--guard == 0) return 3;
    }
    I2C1->DR = reg;
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--guard == 0) return 3;
    }

    I2C1->DR = value;
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--guard == 0) return 4;
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

// Peripheral reset and 100 kHz configuration, shared by init and recovery.
static void i2c1_swrst_recover(void) {
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16u;
    I2C1->CCR = kI2cCcr100k;
    I2C1->TRISE = kI2cTrise100k;
    I2C1->CR1 |= I2C_CR1_PE;
}

// PB6/PB7: open-drain, pull-up, AF4.
static void i2c1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOB->MODER |= (2u << (6u * 2u)) | (2u << (7u * 2u));
    GPIOB->OTYPER |= (1u << 6u) | (1u << 7u);
    GPIOB->PUPDR &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOB->PUPDR |= (1u << (6u * 2u)) | (1u << (7u * 2u));
    GPIOB->AFR[0] &= ~((0xFu << (4u * 6u)) | (0xFu << (4u * 7u)));
    GPIOB->AFR[0] |= (4u << (4u * 6u)) | (4u << (4u * 7u));

    i2c1_swrst_recover();
}

// Switches a PB pin between AF (2) and GPIO output (1); open-drain and pull-up stay as i2c1_init() set them.
static void gpiob_pin_set_mode(uint32_t pin, uint32_t mode) {
    GPIOB->MODER &= ~(3u << (pin * 2u));
    GPIOB->MODER |= (mode << (pin * 2u));
}

volatile uint32_t g_bus_recovery_attempts = 0;
volatile uint32_t g_bus_recovery_freed = 0;

// I2C bus clear (NXP UM10204 3.1.16): a slave holding SDA low is released by clocking SCL up to 9 times as GPIO and
// then sending STOP. SWRST alone resets only our peripheral, not the slave. Needed when a sensor loses power mid-transfer
// while still wired to the bus, and at power-up, when an MPU6050 can come out of reset holding the bus.
// Blocks for roughly 10-15 ms.
//
// A sensor that lost power comes back with PWR_MGMT_1 in SLEEP and CONFIG at its default, while reads still succeed
// with frozen data, so both sensors are woken and their DLPF set again afterwards.
static bool i2c1_bus_recovery(void) {
    g_bus_recovery_attempts = g_bus_recovery_attempts + 1;

    I2C1->CR1 &= ~I2C_CR1_PE; // release peripheral control of the pins
    gpiob_pin_set_mode(6u, 1u); // SCL -> general-purpose output
    gpiob_pin_set_mode(7u, 1u); // SDA -> general-purpose output

    GPIOB->ODR |= (1u << 7u); // let SDA float high (open-drain + pull-up)
    GPIOB->ODR |= (1u << 6u); // SCL high
    delay(2000u);

    bool freed = (GPIOB->IDR & (1u << 7u)) != 0u;
    for (int i = 0; i < 9 && !freed; ++i) {
        GPIOB->ODR &= ~(1u << 6u); // SCL low
        delay(2000u);
        GPIOB->ODR |= (1u << 6u); // SCL high -- clock edge for a stuck slave
        delay(2000u);
        freed = (GPIOB->IDR & (1u << 7u)) != 0u;
    }
    if (freed) {
        g_bus_recovery_freed = g_bus_recovery_freed + 1;
    }

    // STOP: SDA low -> high while SCL is high.
    GPIOB->ODR &= ~(1u << 7u);
    delay(2000u);
    GPIOB->ODR |= (1u << 6u);
    delay(2000u);
    GPIOB->ODR |= (1u << 7u);
    delay(2000u);

    gpiob_pin_set_mode(6u, 2u); // SCL back to AF (I2C1)
    gpiob_pin_set_mode(7u, 2u); // SDA back to AF (I2C1)
    i2c1_swrst_recover();

    mpu6050_write_reg_blocking(kShoulderImuAddr, 0x6Bu, 0x01u);
    mpu6050_write_reg_blocking(kElbowImuAddr, 0x6Bu, 0x01u);
    mpu6050_write_reg_blocking(kShoulderImuAddr, kMpu6050ConfigReg, kDlpfCfg6);
    mpu6050_write_reg_blocking(kElbowImuAddr, kMpu6050ConfigReg, kDlpfCfg6);

    return freed;
}

// Non-blocking multi-byte read. A 14-byte read takes 1.5-2 ms at 100 kHz, longer than one 1 ms tick, so step() checks
// at most one status flag per call and returns at once; the main loop calls it on every pass and a read completes over
// a few ticks without delaying the EMG sampling. The register sequence is the N > 2 receive of the verified blocking
// driver (i2c_mpu6050_hello_main.c).
enum class ImuReadState : uint8_t {
    Idle,
    WaitStart1,
    WaitAddr1,
    WaitRegTxe,
    WaitRegBtf,
    WaitStart2,
    WaitAddr2,
    ReadingMiddle,
    WaitBtfPenultimate,
    WaitBtfLast,
};

class ImuReader {
public:
    explicit ImuReader(uint8_t addr7) : addr7_(addr7) {}

    // start_tick: the 1 kHz tick count now; the timeout is measured in ticks, not in calls to step(), which runs many
    // times per tick. reg/len default to the data read; the once-a-second power check reads 3 bytes from 0x6A
    // (mpu6050_power_check.hpp). len is clamped to 3..14, the range this receive sequence and buf_ support.
    void begin(uint32_t start_tick, uint8_t reg = kImuRegAddr, uint32_t len = kImuReadLen) {
        state_ = ImuReadState::WaitStart1;
        start_tick_ = start_tick;
        byte_index_ = 0;
        reg_ = reg;
        len_ = (len < 3u) ? 3u : ((len > kImuReadLen) ? kImuReadLen : len);
        I2C1->CR1 |= I2C_CR1_START;
    }

    bool is_idle() const { return state_ == ImuReadState::Idle; }
    const uint8_t *buf() const { return buf_; }
    int state_as_int() const { return (int)state_; } // diagnostics only
    // NACKs are per device (this address stopped answering); timeouts are per bus (the bus hung while this reader
    // happened to be active).
    uint32_t nack_count() const { return nack_count_; }
    uint32_t timeout_count() const { return timeout_count_; }
    void reset_counts() { nack_count_ = 0; timeout_count_ = 0; }

    // True once after a NACK or timeout: the device may have reset into SLEEP, so the caller wakes it again on its next
    // completed read instead of trusting the data.
    bool consume_needs_rewake() {
        bool v = needs_rewake_;
        needs_rewake_ = false;
        return v;
    }

    // Returns true on the call that completes the read; buf() then holds it until the next begin().
    bool step(uint32_t current_tick) {
        if (state_ == ImuReadState::Idle) {
            return false;
        }
        if (current_tick - start_tick_ > kImuMaxTicksPerRead) {
            g_imu_state_at_timeout = (int)state_;
            g_sr1_at_timeout = I2C1->SR1;
            g_sr2_at_timeout = I2C1->SR2;
            g_timeout_count = g_timeout_count + 1;
            ++timeout_count_;
            needs_rewake_ = true;
            I2C1->CR1 |= I2C_CR1_STOP;
            i2c1_bus_recovery();
            state_ = ImuReadState::Idle;
            return false;
        }

        switch (state_) {
        case ImuReadState::Idle:
            return false;

        case ImuReadState::WaitStart1:
            if (I2C1->SR1 & I2C_SR1_SB) {
                I2C1->DR = (uint8_t)(addr7_ << 1); // address + W
                state_ = ImuReadState::WaitAddr1;
            }
            return false;

        case ImuReadState::WaitAddr1:
            if (I2C1->SR1 & I2C_SR1_ADDR) {
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                state_ = ImuReadState::WaitRegTxe;
            } else if (I2C1->SR1 & I2C_SR1_AF) {
                I2C1->SR1 &= ~I2C_SR1_AF;
                I2C1->CR1 |= I2C_CR1_STOP;
                state_ = ImuReadState::Idle;
                ++nack_count_;
                needs_rewake_ = true;
            }
            return false;

        case ImuReadState::WaitRegTxe:
            if (I2C1->SR1 & I2C_SR1_TXE) {
                I2C1->DR = reg_;
                state_ = ImuReadState::WaitRegBtf;
            }
            return false;

        case ImuReadState::WaitRegBtf:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 |= I2C_CR1_ACK;
                I2C1->CR1 |= I2C_CR1_START; // repeated START
                state_ = ImuReadState::WaitStart2;
            }
            return false;

        case ImuReadState::WaitStart2:
            if (I2C1->SR1 & I2C_SR1_SB) {
                I2C1->DR = (uint8_t)((addr7_ << 1) | 1u); // address + R
                state_ = ImuReadState::WaitAddr2;
            }
            return false;

        case ImuReadState::WaitAddr2:
            if (I2C1->SR1 & I2C_SR1_ADDR) {
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                byte_index_ = 0;
                state_ = ImuReadState::ReadingMiddle;
            } else if (I2C1->SR1 & I2C_SR1_AF) {
                I2C1->SR1 &= ~I2C_SR1_AF;
                I2C1->CR1 |= I2C_CR1_STOP;
                state_ = ImuReadState::Idle;
                ++nack_count_;
                needs_rewake_ = true;
            }
            return false;

        case ImuReadState::ReadingMiddle:
            if (I2C1->SR1 & I2C_SR1_RXNE) {
                buf_[byte_index_] = (uint8_t)I2C1->DR;
                ++byte_index_;
                if (byte_index_ == len_ - 2u) {
                    state_ = ImuReadState::WaitBtfPenultimate;
                }
            }
            return false;

        case ImuReadState::WaitBtfPenultimate:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                buf_[len_ - 2u] = (uint8_t)I2C1->DR;
                state_ = ImuReadState::WaitBtfLast;
            }
            return false;

        case ImuReadState::WaitBtfLast:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 |= I2C_CR1_STOP;
                buf_[len_ - 1u] = (uint8_t)I2C1->DR;
                state_ = ImuReadState::Idle;
                return true; // full reading complete
            }
            return false;
        }
        return false;
    }

private:
    uint8_t addr7_;
    ImuReadState state_{ImuReadState::Idle};
    uint32_t start_tick_{0};
    uint32_t byte_index_{0};
    uint8_t reg_{kImuRegAddr};
    uint32_t len_{kImuReadLen};
    uint8_t buf_[kImuReadLen]{};
    uint32_t nack_count_{0};
    uint32_t timeout_count_{0};
    bool needs_rewake_{false};
};

static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

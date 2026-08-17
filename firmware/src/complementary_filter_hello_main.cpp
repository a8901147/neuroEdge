// Stage 4b: integrates the already Host-tested
// include/edgeneuro/fusion/complementary_filter.hpp with real MPU6050
// data over I2C1 -- the first time this project's own C++ engine code
// consumes real target-side sensor data, not just a stub/CSV provider.
//
// Isolated from the rest of the EdgeNeuro pipeline on purpose (no
// EdgeNeuro<> instantiation, no EMG/ADC involved here) -- this only
// answers "does ComplementaryFilter behave sensibly fed real IMU data on
// the target," matching the project's one-variable-at-a-time discipline
// (Stage 3a UART alone, 3b ADC alone, 3c/3d combined; 4a I2C+MPU6050
// alone, this stage adds the filter on top of 4a's now-verified read path).
//
// I2C1/MPU6050 register values and the read sequence are unchanged from
// Stage 4a (see i2c_mpu6050_hello_main.cpp's header comment for the full
// RM0368 / InvenSense register-map citations) -- copied here rather than
// shared via a header, matching every other stage file in this directory
// being self-contained.
//
// Unit conversions feeding the filter (verified, not guessed):
//   - Gyro: default FS_SEL=0 -> 131 LSB/(deg/s) (InvenSense PS-MPU-6000A-00
//     Rev 3.4 Section 6.1). Raw/131.0 = deg/s, then x (pi/180) = rad/s,
//     which is what ComplementaryFilter::update() expects.
//   - Accel: default AFS_SEL=0 -> 16384 LSB/g (same doc, Section 6.2).
//     ComplementaryFilter only uses accel via atan2 ratios, so feeding it
//     raw counts instead of true g would still work numerically -- divided
//     here anyway for a value that means something when read off a debugger.
//
// dt: measured, not a delay()-derived guess -- see kDt below and PRD.md
// Phase 1.5 Stage 4b for how complementary_filter_stress_test_main.cpp
// established this loop shape's real ~163ms period. This stage's loop is
// still far coarser than a real control loop (see next paragraph), but is
// fine for a first "does the fusion respond sensibly to real motion" check.
//
// This will NOT get wired into EdgeNeuro<>'s Pipeline -- confirmed
// (PRD.md Section 3, Phase 3 control-architecture note) that IMU
// orientation bypasses Pipeline entirely: Pipeline's window/classify only
// produces a result once every WindowSize samples, which is far too laggy
// for orientation that's supposed to track the user's real arm
// continuously. The real Phase 3 path is closer to what this file already
// does (read sensor, update() every fresh reading, use roll()/pitch()
// directly) than to a Pipeline integration -- future work here is
// tightening the loop timing and making the I2C read non-blocking so it
// doesn't compete with EMG's 1kHz sampling, not routing through Pipeline.

#include <cmath>
#include <cstdint>

#include "edgeneuro/fusion/complementary_filter.hpp"
#include "stm32f4xx.h"

#define MPU6050_ADDR 0x68u // matches Stage 4a; WHO_AM_I check below accepts 0x68 or 0x72
#define LED_PIN 13u

// Diagnostic-only globals, readable via `openocd ... mdw` without relying
// on UART -- see PRD.md Phase 1.5 Stage 4a for why.
volatile uint8_t g_who_am_i = 0xAAu;
volatile int g_who_am_i_result = -1;
volatile int g_wake_result = -1;
volatile int g_loop_count = 0;
volatile int g_last_read_result = -1;
volatile uint32_t g_sr1_at_af = 0xFFFFFFFFu; // SR1 snapshot at the moment AF is detected, see Stage 4a

static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

// --- USART2 (unchanged from every prior stage) ---

static void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));

    USART2->BRR = 0x0683u;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void usart2_send_byte(uint8_t byte) {
    while (!(USART2->SR & USART_SR_TXE)) {
    }
    USART2->DR = byte;
}

static void usart2_send_string(const char *s) {
    while (*s) {
        usart2_send_byte((uint8_t)*s++);
    }
}

static void usart2_send_int(int32_t value) {
    char digits[12];
    int n = 0;
    uint32_t magnitude;
    if (value < 0) {
        usart2_send_byte('-');
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }
    if (magnitude == 0) {
        usart2_send_byte('0');
        return;
    }
    while (magnitude > 0 && n < 12) {
        digits[n++] = (char)('0' + (magnitude % 10u));
        magnitude /= 10u;
    }
    while (n > 0) {
        usart2_send_byte((uint8_t)digits[--n]);
    }
}

// No printf/float formatting in this bare-metal build -- prints degrees
// x10 as an integer (e.g. 45.3 deg -> "453", caller adds the decimal
// point) so one decimal digit of precision survives without pulling in
// a float-to-string routine.
static void usart2_send_deg_x10(float radians) {
    const float degrees = radians * (180.0f / 3.14159265f);
    const int32_t tenths = (int32_t)(degrees * 10.0f);
    usart2_send_int(tenths);
}

// --- I2C1 master driver (PB6=SCL, PB7=SDA) -- unchanged from Stage 4a ---

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

    // If a previous transaction was interrupted mid-sequence (e.g. by a
    // debug reset), the peripheral's internal state machine -- including
    // the BUSY flag in SR2 -- can stay latched even once SDA/SCL are both
    // idle-high again. SWRST forces a full reset of that state machine
    // (RM0368 18.6.1); safe to do unconditionally on every init.
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16u;
    I2C1->CCR = 0x50u;
    I2C1->TRISE = 0x11u;
    I2C1->CR1 |= I2C_CR1_PE;
}

static int i2c1_start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    uint32_t guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--guard == 0) return 1;
    }
    return 0;
}

static int i2c1_send_address(uint8_t addr7, int read) {
    I2C1->DR = (uint8_t)((addr7 << 1) | (read ? 1u : 0u));
    uint32_t guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            g_sr1_at_af = I2C1->SR1;
            I2C1->SR1 &= ~I2C_SR1_AF;
            return 1;
        }
        if (--guard == 0) return 1;
    }
    return 0;
}

static void i2c1_clear_addr(void) {
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static int i2c1_write_byte(uint8_t data) {
    uint32_t guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (--guard == 0) return 1;
    }
    I2C1->DR = data;
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--guard == 0) return 1;
    }
    return 0;
}

static void i2c1_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

static int mpu6050_write_reg(uint8_t reg, uint8_t value) {
    if (i2c1_start()) return 1;
    if (i2c1_send_address(MPU6050_ADDR, 0)) { i2c1_stop(); return 1; }
    i2c1_clear_addr();
    if (i2c1_write_byte(reg)) { i2c1_stop(); return 1; }
    if (i2c1_write_byte(value)) { i2c1_stop(); return 1; }
    i2c1_stop();
    return 0;
}

// See Stage 4a (i2c_mpu6050_hello_main.c) for why the tail of a multi-byte
// read must be timed off BTF, not RXNE -- this is copied verbatim from
// the version that was empirically confirmed working on real hardware.
static int mpu6050_read_regs(uint8_t reg, uint8_t *out, uint32_t len) {
    if (i2c1_start()) return 1;
    if (i2c1_send_address(MPU6050_ADDR, 0)) { i2c1_stop(); return 2; }
    i2c1_clear_addr();
    if (i2c1_write_byte(reg)) { i2c1_stop(); return 3; }

    I2C1->CR1 |= I2C_CR1_ACK;
    if (i2c1_start()) { i2c1_stop(); return 4; }
    if (i2c1_send_address(MPU6050_ADDR, 1)) { i2c1_stop(); return 5; }

    if (len == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c1_clear_addr();
        I2C1->CR1 |= I2C_CR1_STOP;
        uint32_t guard = 100000u;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (--guard == 0) return 6;
        }
        out[0] = (uint8_t)I2C1->DR;
        return 0;
    }

    i2c1_clear_addr();

    for (uint32_t i = 0; i + 2 < len; ++i) {
        uint32_t guard = 100000u;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (--guard == 0) return 6;
        }
        out[i] = (uint8_t)I2C1->DR;
    }

    {
        uint32_t guard = 100000u;
        while (!(I2C1->SR1 & I2C_SR1_BTF)) {
            if (--guard == 0) return 6;
        }
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    out[len - 2] = (uint8_t)I2C1->DR;

    {
        uint32_t guard = 100000u;
        while (!(I2C1->SR1 & I2C_SR1_BTF)) {
            if (--guard == 0) return 6;
        }
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    out[len - 1] = (uint8_t)I2C1->DR;
    return 0;
}

static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void blink_code(int code) {
    while (1) {
        for (int i = 0; i < code; ++i) {
            GPIOC->ODR &= ~(1u << LED_PIN);
            delay(150000u);
            GPIOC->ODR |= (1u << LED_PIN);
            delay(150000u);
        }
        delay(1200000u);
    }
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u));

    usart2_init();
    i2c1_init();

    uint8_t who_am_i = 0;
    g_who_am_i_result = mpu6050_read_regs(0x75u, &who_am_i, 1);
    g_who_am_i = who_am_i;
    if (g_who_am_i_result != 0) {
        usart2_send_string("MPU6050 WHO_AM_I read FAILED\r\n");
        blink_code(1);
    }
    if (who_am_i != 0x68u && who_am_i != 0x72u) { // see Stage 4a: this board reports 0x72
        usart2_send_string("MPU6050 WHO_AM_I MISMATCH, got 0x");
        usart2_send_int(who_am_i);
        usart2_send_string("\r\n");
        blink_code(2);
    }

    g_wake_result = mpu6050_write_reg(0x6Bu, 0x01u); // PWR_MGMT_1: wake, CLKSEL=1
    if (g_wake_result != 0) {
        usart2_send_string("MPU6050 wake write FAILED\r\n");
        blink_code(3);
    }
    delay(1000000u);

    // Measured, not estimated: complementary_filter_stress_test_main.cpp
    // runs this same delay(400000u)-paced loop shape (I2C transaction +
    // two UART prints per iteration) and wall-clock timing over a 60s/367-
    // iteration run gave 163.4ms/iteration, not the ~250ms a delay()-only
    // estimate would suggest (see PRD.md Phase 1.5 Stage 4b). This loop's
    // I2C read is longer (14 bytes vs. the stress test's 1-byte LCD
    // write), so re-measure directly once a working MPU6050 is attached;
    // 0.163f is a much closer starting point than the old guess either way.
    constexpr float kDt = 0.163f;
    constexpr float kDegToRad = 3.14159265f / 180.0f;
    constexpr float kGyroLsbPerDegPerSec = 131.0f;  // FS_SEL=0
    constexpr float kAccelLsbPerG = 16384.0f;       // AFS_SEL=0

    edgeneuro::ComplementaryFilter<float> filter(0.98f, kDt);

    // Seed from the first real accelerometer reading rather than starting
    // at roll=pitch=0 -- this is exactly the cold-start transient found
    // and fixed during Host-side validation against EMG-EPN-612 data (see
    // PRD.md Phase 1.5's fusion section); it applies here too.
    {
        uint8_t raw[6];
        if (mpu6050_read_regs(0x3Bu, raw, 6) == 0) {
            const float ax = (float)be16(&raw[0]) / kAccelLsbPerG;
            const float ay = (float)be16(&raw[2]) / kAccelLsbPerG;
            const float az = (float)be16(&raw[4]) / kAccelLsbPerG;
            filter.initialize(ax, ay, az);
        }
    }

    while (1) {
        uint8_t raw[14];
        g_loop_count = g_loop_count + 1;
        g_last_read_result = mpu6050_read_regs(0x3Bu, raw, 14);
        if (g_last_read_result == 0) {
            const float ax = (float)be16(&raw[0]) / kAccelLsbPerG;
            const float ay = (float)be16(&raw[2]) / kAccelLsbPerG;
            const float az = (float)be16(&raw[4]) / kAccelLsbPerG;
            const float gx = (float)be16(&raw[8]) / kGyroLsbPerDegPerSec * kDegToRad;
            const float gy = (float)be16(&raw[10]) / kGyroLsbPerDegPerSec * kDegToRad;

            filter.update(gx, gy, ax, ay, az);

            usart2_send_string("roll_x10=");
            usart2_send_deg_x10(filter.roll());
            usart2_send_string(" pitch_x10=");
            usart2_send_deg_x10(filter.pitch());
            usart2_send_string("\r\n");

            GPIOC->ODR ^= (1u << LED_PIN);
        } else {
            usart2_send_string("MPU6050 read FAILED\r\n");
        }
        delay(400000u);
    }
}

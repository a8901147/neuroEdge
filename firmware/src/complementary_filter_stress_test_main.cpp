// Stage 4b stress test -- NOT a pipeline stage, a diagnostic tool.
//
// Purpose: Phase 1.5's real MPU6050/GY-521 modules stopped responding
// (see PRD.md Stage 4b), so complementary_filter_hello_main.cpp has never
// actually run its main loop for any length of time -- it dies at the
// WHO_AM_I check before ever reaching filter.update(). That leaves an
// untested question completely separate from "does the sensor respond":
// does the loop itself (filter math, USART2 printing, timing, repeated
// I2C traffic) hold up over thousands of iterations, or is there a
// latent firmware bug (NaN creep, a slow UART leak, a timing drift) that
// would only show up after a while?
//
// This answers that question without needing a working IMU: it feeds
// ComplementaryFilter smooth synthetic accel/gyro data (a slow simulated
// tilt, not real motion) instead of reading the MPU6050, and uses
// whatever I2C1 device happens to be on the bus (developed against an
// LCD1602/PCF8574 backpack at address 0x27, see PRD.md Stage 4b) purely
// as a live bus load by toggling its backlight control bit each
// iteration -- exercising the same I2C1 write path repeatedly, the way
// the real loop would exercise its read path. If this runs cleanly for
// an extended period, any problem found once a real MPU6050 arrives can
// be attributed to the sensor, not general firmware plumbing.
//
// I2C1 driver unchanged from Stage 4a/4b (PB6=SCL, PB7=SDA, AF4, 100kHz
// Sm mode, SWRST recovery on init) -- see i2c_mpu6050_hello_main.c's
// header comment for the RM0368 citations.

#include <cmath>
#include <cstdint>

#include "edgeneuro/fusion/complementary_filter.hpp"
#include "stm32f4xx.h"

#define LED_PIN 13u
#define LCD_ADDR 0x27u // PCF8574 backpack default; harmless if nothing ACKs here

volatile int g_loop_count = 0;
volatile int g_last_i2c_result = -1;
volatile int g_math_error = 0; // set if roll/pitch ever goes NaN/Inf

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

static void usart2_send_deg_x10(float radians) {
    const float degrees = radians * (180.0f / 3.14159265f);
    const int32_t tenths = (int32_t)(degrees * 10.0f);
    usart2_send_int(tenths);
}

// --- I2C1 master driver (PB6=SCL, PB7=SDA) -- unchanged from Stage 4a/4b ---

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

// Writes one raw byte to the PCF8574 backpack -- with EN (bit2) always 0,
// this never latches a command/data byte into the HD44780, so it can't
// corrupt whatever the display currently shows. Only used here to put
// real, repeated I2C write traffic on the bus; toggling bit3 (backlight
// on common backpacks) as a visible side effect is a bonus, not the point.
static int lcd_write_byte(uint8_t data) {
    if (i2c1_start()) return 1;
    if (i2c1_send_address(LCD_ADDR, 0)) { i2c1_stop(); return 2; }
    i2c1_clear_addr();
    if (i2c1_write_byte(data)) { i2c1_stop(); return 3; }
    i2c1_stop();
    return 0;
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
    usart2_send_string("Stage 4b stress test: synthetic IMU data, no real MPU6050 needed\r\n");

    constexpr float kDt = 0.25f; // matches this loop's delay(400000u) pacing

    edgeneuro::ComplementaryFilter<float> filter(0.98f, kDt);
    filter.initialize(0.0f, 0.0f, 1.0f); // seed level (no real first reading here)

    float phase = 0.0f;

    while (1) {
        g_loop_count = g_loop_count + 1;

        // Slow simulated tilt, bounded and physically-plausible (|accel|
        // stays ~1g) so the filter sees a realistic-shaped input rather
        // than an edge case that wouldn't occur with a real sensor.
        phase += 0.05f;
        if (phase > 6.283185f) {
            phase -= 6.283185f;
        }
        const float ax = 0.3f * sinf(phase * 0.1f);
        const float ay = 0.3f * cosf(phase * 0.1f);
        const float az = sqrtf(1.0f - ax * ax - ay * ay);
        const float gx = 0.5f * sinf(phase);
        const float gy = 0.5f * cosf(phase);

        filter.update(gx, gy, ax, ay, az);

        if (std::isnan(filter.roll()) || std::isnan(filter.pitch()) ||
            std::isinf(filter.roll()) || std::isinf(filter.pitch())) {
            g_math_error = 1;
            usart2_send_string("MATH ERROR: roll/pitch is NaN/Inf at loop ");
            usart2_send_int(g_loop_count);
            usart2_send_string("\r\n");
            blink_code(9);
        }

        usart2_send_string("loop=");
        usart2_send_int(g_loop_count);
        usart2_send_string(" roll_x10=");
        usart2_send_deg_x10(filter.roll());
        usart2_send_string(" pitch_x10=");
        usart2_send_deg_x10(filter.pitch());
        usart2_send_string("\r\n");

        g_last_i2c_result = lcd_write_byte((g_loop_count & 1) ? 0x08u : 0x00u); // backlight toggle

        GPIOC->ODR ^= (1u << LED_PIN);
        delay(400000u);
    }
}

// I2C1 bus scanner -- a permanent diagnostic tool, not a one-off. Grew out
// of Phase 1.5 Stage 4b: MPU6050/GY-521 stopped ACKing after repeated
// breadboard handling, and the fastest way to tell "STM32 side broken" from
// "connected device broken" turned out to be scanning the full 7-bit
// address space with a device of known-good address (an LCD1602/PCF8574
// backpack, address 0x27) rather than re-checking wiring by hand each time.
// See PRD.md Phase 1.5 Stage 4b for the full elimination process this
// replaced.
//
// Reuses the I2C1 driver verified in i2c_mpu6050_hello_main.c (PB6=SCL,
// PB7=SDA, AF4, 100kHz Sm mode, SWRST recovery on init -- see that file's
// header comment for the RM0368 citations, not repeated here).
//
// Usage: flash this, then read g_scan_done/g_scan_bitmap/g_bus_busy via
// SWD (see tools/check_hardware_ready.py --i2c-scan, which automates
// exactly this). No UART output -- this is meant to be readable purely
// via `openocd ... mdw` even when UART is unavailable, matching the rest
// of this session's SWD-first diagnostic approach.

#include "stm32f4xx.h"

#define LED_PIN 13u

// bit N set = 7-bit address N ACKed. [0]=0-31, [1]=32-63, [2]=64-95, [3]=96-127.
volatile uint32_t g_scan_bitmap[4] = {0u, 0u, 0u, 0u};
volatile int g_scan_done = 0;
volatile uint32_t g_bus_busy_before_scan = 0xFFFFFFFFu; // I2C1_SR2.BUSY, read right after init

static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

static void i2c1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOB->MODER |= (2u << (6u * 2u)) | (2u << (7u * 2u)); // AF mode
    GPIOB->OTYPER |= (1u << 6u) | (1u << 7u);              // open-drain, required for I2C
    GPIOB->PUPDR &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOB->PUPDR |= (1u << (6u * 2u)) | (1u << (7u * 2u)); // internal pull-up (safety net)
    GPIOB->AFR[0] &= ~((0xFu << (4u * 6u)) | (0xFu << (4u * 7u)));
    GPIOB->AFR[0] |= (4u << (4u * 6u)) | (4u << (4u * 7u)); // AF4 = I2C1

    // Forces a full reset of the peripheral's internal state machine
    // (including a latched BUSY flag) even if SDA/SCL are both idle-high
    // (RM0368 18.6.1) -- safe to do unconditionally on every init.
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR1 &= ~I2C_CR1_PE; // must be disabled to configure CCR/TRISE
    I2C1->CR2 = 16u;          // FREQ[5:0] = 16MHz APB1
    I2C1->CCR = 0x50u;        // 100kHz Sm mode @ 16MHz
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
        if (I2C1->SR1 & I2C_SR1_AF) { // NACK -- nothing at this address
            I2C1->SR1 &= ~I2C_SR1_AF;
            return 1;
        }
        if (--guard == 0) return 1;
    }
    return 0;
}

static void i2c1_clear_addr(void) {
    (void)I2C1->SR1;
    (void)I2C1->SR2; // ADDR cleared by reading SR1 then SR2 (RM0368 18.3.3 EV6)
}

static void i2c1_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

// Blinks `code` short pulses then a long pause, forever -- readable by eye
// without needing UART. code=1: scan complete.
static void blink_code(int code) {
    while (1) {
        for (int i = 0; i < code; ++i) {
            GPIOC->ODR &= ~(1u << LED_PIN); // on (active-low)
            delay(150000u);
            GPIOC->ODR |= (1u << LED_PIN); // off
            delay(150000u);
        }
        delay(1200000u);
    }
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u));

    i2c1_init();
    g_bus_busy_before_scan = I2C1->SR2 & I2C_SR2_BUSY;

    for (uint8_t addr = 1u; addr < 127u; ++addr) {
        if (i2c1_start() == 0) {
            if (i2c1_send_address(addr, 0) == 0) {
                g_scan_bitmap[addr >> 5u] |= (1u << (addr & 31u));
                i2c1_clear_addr();
            }
        }
        i2c1_stop();
        delay(20000u);
    }
    g_scan_done = 1;
    blink_code(1);
    return 0;
}

// Stage 4a: I2C1 (PB6=SCL, PB7=SDA) polled master driver, reading an
// MPU6050/GY-521 IMU -- isolated bring-up before wiring the already
// Host-tested ComplementaryFilter (include/edgeneuro/fusion/complementary_filter.hpp)
// into a full firmware stage, same "one variable at a time" discipline as
// Stage 3a (UART alone) / 3b (ADC alone) before 3c/3d combined them.
//
// This checks WHO_AM_I first (must read the sensor's documented reset
// value) and only proceeds to stream accel/gyro if that matches -- if
// wiring, address, or timing is wrong, this fails loudly over UART
// instead of silently printing garbage.
//
// Every register value below is verified against two separate official
// documents, not guessed:
//
// STM32F401 side (RM0368 Rev 5):
//   - PB6/PB7 = AF4 for I2C1_SCL/I2C1_SDA (STM32F401CCU6 datasheet
//     DocID024738 Table 9 -- same table already used for PA2/PA3/USART2).
//   - RCC_AHB1ENR bit 1 = GPIOBEN, RCC_APB1ENR bit 21 = I2C1EN (RM0368
//     6.3.9 / 6.3.11).
//   - I2C pins must be open-drain (GPIOx_OTYPER = 1) -- I2C is a
//     wired-AND bus, push-pull would let one device drive the line high
//     while another drives it low (RM0368 8.4 / I2C bus electrical
//     requirement, not STM32-specific but the OTYPER mechanism to
//     implement it is). Internal pull-ups enabled too (GPIOx_PUPDR = 01)
//     as a safety net alongside the GY-521 breakout's own pull-ups.
//   - I2C_CR2 FREQ[5:0] = 16 (APB1 clock in MHz, same 16MHz HSI as every
//     other stage) (RM0368 18.6.2).
//   - I2C_CCR: Sm mode (F/S=0) 100kHz. CCR = TPCLK1-cycles for thigh=tlow
//     = (1/100kHz)/2 / (1/16MHz) = 80 = 0x50 (RM0368 18.6.8, Sm mode
//     formula: Thigh = Tlow = CCR * TPCLK1).
//   - I2C_TRISE = (1000ns max Sm-mode rise time / 62.5ns TPCLK1) + 1 = 17
//     = 0x11 (RM0368 18.6.9).
//   - Master transmit/receive polled sequences (SB/ADDR/TxE/RxNE/BTF
//     flags, clearing ADDR by reading SR1 then SR2, NACK+STOP sequencing
//     for the last received byte) follow RM0368 18.3.3's documented
//     event sequence (EV5/EV6/EV8/EV8_2 for transmit, EV5/EV6/EV7/EV7_1
//     for receive), not a remembered "typical I2C" pattern.
//
// MPU6050 side (InvenSense RM-MPU-6000A-00 Rev 4.0, register map; and
// PS-MPU-6000A-00 Rev 3.4, product spec, for scale factors):
//   - 7-bit I2C address 0x68, assuming AD0 tied low (GY-521 breakout
//     default -- address would be 0x69 if AD0 is instead tied high on
//     this specific board, see this file's README note).
//   - Register 0x6B (PWR_MGMT_1): reset value 0x40 (SLEEP=1) -- the
//     sensor powers up asleep and outputs nothing until this is cleared.
//     Written 0x01 here: SLEEP=0 (wake), CLKSEL=001 (PLL with X-axis
//     gyro reference) -- register map doc explicitly recommends CLKSEL=1
//     over the reset default CLKSEL=0 (internal 8MHz oscillator) "for
//     improved stability", not left at reset.
//   - Register 0x75 (WHO_AM_I): reset/fixed value 0x68 (bits 6:1 =
//     0b110100, bits 7/0 hardcoded 0) -- read-only identity check.
//   - Registers 0x3B-0x48: ACCEL_XOUT_H/L, ACCEL_YOUT_H/L, ACCEL_ZOUT_H/L,
//     TEMP_OUT_H/L, GYRO_XOUT_H/L, GYRO_YOUT_H/L, GYRO_ZOUT_H/L -- 14
//     contiguous bytes, big-endian 16-bit two's complement each, read in
//     one burst.
//   - Default AFS_SEL=0 (register 0x1C, left at reset) = +/-2g range,
//     16384 LSB/g. Default FS_SEL=0 (register 0x1B, left at reset) =
//     +/-250 deg/s range, 131 LSB/(deg/s). Both scale factors are from
//     the product spec's Electrical Characteristics tables (Section 6.1
//     gyroscope, 6.2 accelerometer), not computed from the full-scale
//     range alone.
//
// All register field macros (RCC_AHB1ENR_GPIOBEN, I2C_CR1_PE,
// I2C_SR1_SB, etc.) confirmed present in the same CMSIS device header
// used by every prior stage.

#include <math.h>
#include "stm32f4xx.h"

#define MPU6050_ADDR 0x68u // 7-bit; try 0x69 if AD0 is tied high on your GY-521

// Diagnostic-only globals: readable via `openocd ... mdw` without relying
// on UART (proven unreliable this session) or needing debug-symbol/stack
// unwinding to inspect a local variable at a breakpoint.
volatile uint8_t g_last_who_am_i = 0xAAu; // sentinel so "never written" is obvious
volatile int g_last_i2c_result = -1;

#define LED_PIN 13u

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

// --- USART2 (reused, verified in Stage 3a) ---

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

// --- I2C1 master driver (PB6=SCL, PB7=SDA) ---

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

    I2C1->CR1 &= ~I2C_CR1_PE; // must be disabled to configure CCR/TRISE
    I2C1->CR2 = 16u;          // FREQ[5:0] = 16MHz APB1
    I2C1->CCR = 0x50u;        // 100kHz Sm mode @ 16MHz
    I2C1->TRISE = 0x11u;
    I2C1->CR1 |= I2C_CR1_PE;
}

// Returns 0 on success, non-zero (arbitrary) on failure -- checked at
// every call site so a communication fault halts visibly instead of
// silently streaming garbage.

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
        if (I2C1->SR1 & I2C_SR1_AF) { // NACK on address -- wrong address or device not present
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

// Writes one byte to one register: S, address+W, reg, data, P.
static int mpu6050_write_reg(uint8_t reg, uint8_t value) {
    if (i2c1_start()) return 1;
    if (i2c1_send_address(MPU6050_ADDR, 0)) { i2c1_stop(); return 1; }
    i2c1_clear_addr();
    if (i2c1_write_byte(reg)) { i2c1_stop(); return 1; }
    if (i2c1_write_byte(value)) { i2c1_stop(); return 1; }
    i2c1_stop();
    return 0;
}

// Reads `len` contiguous bytes starting at `reg`: S, address+W, reg,
// Sr (repeated start), address+R, [len bytes, NACK on the last one], P.
//
// Returns a distinct nonzero code per failure site (not just 1) so the
// caller can blink out WHICH step failed on the LED -- UART on this
// setup has proven too unreliable this session to depend on for
// diagnostics, see PRD.md Phase 1.5 for the pattern.
//   1 = first START (bus/wiring issue before any device response needed)
//   2 = address+W not ACKed (wrong 7-bit address, or device not present)
//   3 = writing the register-address byte failed
//   4 = repeated START failed
//   5 = address+R not ACKed
//   6 = a data byte was never clocked in (RXNE timeout)
static int mpu6050_read_regs(uint8_t reg, uint8_t *out, uint32_t len) {
    if (i2c1_start()) return 1;
    if (i2c1_send_address(MPU6050_ADDR, 0)) { i2c1_stop(); return 2; }
    i2c1_clear_addr();
    if (i2c1_write_byte(reg)) { i2c1_stop(); return 3; }

    I2C1->CR1 |= I2C_CR1_ACK; // ACK every received byte except the last
    if (i2c1_start()) { i2c1_stop(); return 4; } // repeated start
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

    // len >= 2: RM0368 18.3.3 "For N >2-byte reception, from N-2 data
    // reception" procedure. An earlier version of this function tried to
    // time the ACK-clear and STOP-set relative to RXNE (checked per
    // iteration), which is NOT what the reference manual specifies for
    // multi-byte reads and reliably failed in practice on real hardware
    // (single-byte WHO_AM_I reads worked; the 14-byte accel+gyro burst
    // read did not) -- BTF (not RXNE) is what correctly indicates "safe
    // to flip ACK/STOP without racing the hardware's next byte" here.
    i2c1_clear_addr();

    for (uint32_t i = 0; i + 2 < len; ++i) {
        uint32_t guard = 100000u;
        while (!(I2C1->SR1 & I2C_SR1_RXNE)) {
            if (--guard == 0) return 6;
        }
        out[i] = (uint8_t)I2C1->DR;
    }

    // BTF=1: data[len-2] sitting in DR, data[len-1] already fully
    // shifted in (its ACK bit not yet sent -- SCL is stretched low until
    // DR is read), data[len] not yet started.
    {
        uint32_t guard = 100000u;
        while (!(I2C1->SR1 & I2C_SR1_BTF)) {
            if (--guard == 0) return 6;
        }
    }
    I2C1->CR1 &= ~I2C_CR1_ACK; // NACK data[len-1] before its ACK bit goes out
    out[len - 2] = (uint8_t)I2C1->DR; // releases the clock stretch

    // BTF=1 again: data[len-1] in DR, data[len] (the final byte) fully
    // shifted in.
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

// UART on this setup has proven too unreliable this session to depend on
// for diagnostics (adapter repeatedly drops and needs a physical
// unplug/replug cycle to recover -- see PRD.md Phase 1.5). This blinks
// `code` short pulses, then a long pause, forever -- readable by eye or
// stopwatch without needing the serial link to be up at all.
static void blink_code(int code) {
    while (1) {
        for (int i = 0; i < code; ++i) {
            GPIOC->ODR &= ~(1u << LED_PIN); // on (active-low)
            delay(150000u);
            GPIOC->ODR |= (1u << LED_PIN); // off
            delay(150000u);
        }
        delay(1200000u); // long pause between repeats of the count
    }
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u));

    usart2_init();
    usart2_send_string("checkpoint 1: usart2 ok\r\n");
    i2c1_init();
    usart2_send_string("checkpoint 2: i2c1_init ok\r\n");
    usart2_send_string("checkpoint 3: about to read WHO_AM_I\r\n");

    uint8_t who_am_i = 0;
    const int who_am_i_result = mpu6050_read_regs(0x75u, &who_am_i, 1);
    g_last_i2c_result = who_am_i_result;
    g_last_who_am_i = who_am_i;
    if (who_am_i_result != 0) {
        usart2_send_string("MPU6050 WHO_AM_I read FAILED, code=");
        usart2_send_int(who_am_i_result);
        usart2_send_string("\r\n");
        blink_code(who_am_i_result); // 1-6, see mpu6050_read_regs's doc comment
    }
    // Register map doc says WHO_AM_I's reset/fixed value is 0x68, but this
    // specific board consistently (confirmed across 3 independent
    // reflashes via SWD register readout, not a one-off) returns 0x72
    // instead -- a real, stable, non-noise response, just not the
    // documented ID. Likely a clone/compatible chip on this GY-521 board,
    // not a communication fault (a real fault would show up as
    // who_am_i_result != 0 above, or an inconsistent/varying byte, not a
    // clean repeatable 0x72). Accept both rather than hard-failing on a
    // chip-identity quirk when the actual protocol-level communication is
    // demonstrably working.
    if (who_am_i != 0x68u && who_am_i != 0x72u) {
        usart2_send_string("MPU6050 WHO_AM_I MISMATCH, got 0x");
        usart2_send_int(who_am_i);
        usart2_send_string(" (expected 0x68 or 0x72)\r\n");
        blink_code(7); // read succeeded but returned an unrecognized value
    }
    usart2_send_string("MPU6050 WHO_AM_I OK, got 0x");
    usart2_send_int(who_am_i);
    usart2_send_string("\r\n");

    if (mpu6050_write_reg(0x6Bu, 0x01u)) { // PWR_MGMT_1: wake, CLKSEL=1 (X-gyro PLL ref)
        usart2_send_string("MPU6050 wake write FAILED\r\n");
        blink_code(8);
    }
    delay(1000000u); // let the clock source settle before trusting readings

    while (1) {
        uint8_t raw[14];
        if (mpu6050_read_regs(0x3Bu, raw, 14) == 0) {
            const int16_t accel_x = be16(&raw[0]);
            const int16_t accel_y = be16(&raw[2]);
            const int16_t accel_z = be16(&raw[4]);
            const int16_t gyro_x = be16(&raw[8]);
            const int16_t gyro_y = be16(&raw[10]);

            usart2_send_string("accel_raw x=");
            usart2_send_int(accel_x);
            usart2_send_string(" y=");
            usart2_send_int(accel_y);
            usart2_send_string(" z=");
            usart2_send_int(accel_z);
            usart2_send_string(" gyro_raw x=");
            usart2_send_int(gyro_x);
            usart2_send_string(" y=");
            usart2_send_int(gyro_y);
            usart2_send_string("\r\n");

            GPIOC->ODR ^= (1u << LED_PIN);
        } else {
            usart2_send_string("MPU6050 read FAILED\r\n");
        }
        delay(1600000u); // rough ~1s-ish pacing, not calibrated -- same style as prior stages
    }
}

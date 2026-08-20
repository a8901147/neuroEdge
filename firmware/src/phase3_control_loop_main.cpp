// Stage 5b: the real Phase 3 control loop shape -- EMG (Stage 5a,
// verified working on real hardware) and IMU orientation sharing one
// 1kHz main loop, IMU read as a non-blocking state machine instead of
// the blocking mpu6050_read_regs() every prior IMU stage used.
//
// Why non-blocking is mandatory here, not just nicer: EMG's TIM2+ADC1
// trigger fires every 1ms (hardware-verified, PRD.md Stage 3c/3d). A
// full 14-byte I2C read at 100kHz takes ~1.5-2ms -- longer than one tick
// -- so doing it as one blocking call would stall EMG sampling for more
// than a full tick period every time. Instead, mpu6050_read_step() below
// advances the read by checking at most one hardware flag per call and
// returning immediately either way; the main loop calls it once per EMG
// tick, and a full reading completes opportunistically across however
// many ticks the bus actually needs (in practice 2-4 ticks) without ever
// blocking any single tick.
//
// dt for ComplementaryFilter::update() is real elapsed ticks since the
// last completed reading (kDtPerTick=0.001f, from the same TIM2-verified
// exact period Stage 5a's kDt used) -- not a guess, and not the same
// value every call, since reads don't complete on a fixed schedule.
//
// kImuTargetAddr defaults to the LCD1602/PCF8574 backpack's address
// (0x27), NOT the real MPU6050 address (0x68) -- both real MPU6050 units
// are still confirmed dead (PRD.md Stage 4b) and can't be used to
// validate this state machine at all. The LCD ACKs address+W, accepts
// any byte as if it were a register address (PCF8574 doesn't have
// registers, it just latches GPIO state), and ACKs address+R, returning
// whatever's on its input pins -- not real sensor data, but real I2C bus
// timing that exercises the exact same START/ADDR/TXE/BTF/RXNE/STOP
// sequence, including the multi-byte-read BTF tail timing that was the
// actual bug in Stage 4a's original blocking implementation. Once a
// working MPU6050 arrives, flip kImuTargetAddr to 0x68 and the state
// machine needs no other changes.

// Command trajectory smoothing on the IMU side: a light low-pass on
// roll()/pitch() to remove residual sample-to-sample jitter before it
// reaches an actuator setpoint -- distinct from ComplementaryFilter's own
// smoothing, which turns a noisy *measurement* into a believable angle,
// not a noisy *already-fused* angle into a smoother command (see
// include/edgeneuro/control/slew_rate_limiter.hpp's header comment for
// the same distinction on the EMG side). Reuses the project's existing
// IirFilter rather than a new component, per PRD.md Stage 5b's control-
// architecture note.
//
// Configured as a single-pole exponential moving average expressed as
// IirFilter's biquad (b0=kSmoothAlpha, a1=kSmoothAlpha-1, everything else
// 0): y[n] = alpha*x[n] + (1-alpha)*y[n-1]. kSmoothAlpha=0.5 is a
// starting placeholder, not a derived cutoff frequency -- IMU reads
// complete irregularly (see ImuReader below) so the real sample rate
// isn't fixed yet, making a proper frequency-domain design premature.
// Retune once real MPU6050 jitter is available to look at (LCD1602's
// stand-in readings are constant, not jittery, so there's nothing to
// tune against yet -- see PRD.md Stage 5b).
static constexpr float kSmoothAlpha = 0.5f;

#include <cstdint>

#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/slew_rate_limiter.hpp"
#include "edgeneuro/control/threshold_calibrator.hpp"
#include "edgeneuro/filters/iir_filter.hpp"
#include "edgeneuro/fusion/complementary_filter.hpp"
#include "stm32f4xx.h"

#define LED_PIN 13u

// --- EMG side (unchanged constants from Stage 5a) ---
static constexpr bool kCalibrationEnabled = false;
static constexpr float kFallbackThreshold = 2037.0f;
static constexpr float kOnDuration = 0.15f;
static constexpr float kOffDuration = 0.15f;
static constexpr float kSlewRate = 5.0f;
static constexpr float kDtPerTick = 0.001f; // TIM2-verified exact 1kHz
static constexpr uint32_t kCalibrationSamples = 3000u;

// --- IMU side ---
static constexpr uint8_t kImuTargetAddr = 0x27u; // TEMP: LCD1602 stand-in, see header comment. Real MPU6050 = 0x68.
static constexpr uint8_t kImuRegAddr = 0x3Bu;    // ACCEL_XOUT_H -- meaningless against the LCD, kept for protocol shape
static constexpr uint32_t kImuReadLen = 14u;
static constexpr uint32_t kImuMaxTicksPerRead = 50u; // abort+retry a read stuck > 50ms

// Diagnostic-only globals, readable via `openocd ... mdw` -- see PRD.md
// Stage 5b debugging notes.
volatile int g_imu_state_at_timeout = -1;
volatile uint32_t g_sr1_at_timeout = 0xFFFFFFFFu;
volatile uint32_t g_sr2_at_timeout = 0xFFFFFFFFu;
volatile uint32_t g_timeout_count = 0;

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
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static uint8_t usart2_recv_byte(void) {
    while (!(USART2->SR & USART_SR_RXNE)) {
    }
    return (uint8_t)USART2->DR;
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

static void usart2_send_uint(uint32_t value) {
    char digits[10];
    int n = 0;
    if (value == 0) {
        usart2_send_byte('0');
        return;
    }
    while (value > 0 && n < 10) {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    }
    while (n > 0) {
        usart2_send_byte((uint8_t)digits[--n]);
    }
}

static void usart2_send_int(int32_t value) {
    if (value < 0) {
        usart2_send_byte('-');
        usart2_send_uint((uint32_t)(-value));
    } else {
        usart2_send_uint((uint32_t)value);
    }
}

static void usart2_send_float_x1000(float value) {
    usart2_send_int((int32_t)(value * 1000.0f));
}

// --- EMG: ADC1 channel 0 (PA0) via TIM2 TRGO, verified Stage 3c/3d ---

static void gpioa_pa0_analog(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (0u * 2u));
    GPIOA->MODER |= (3u << (0u * 2u));
}

static void tim2_init_1khz_trgo(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15u;
    TIM2->ARR = 999u;
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void adc1_init_timer_triggered(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3 &= ~ADC_SQR3_SQ1;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(10000u);
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL)) |
                ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
}

// --- IMU: I2C1 (PB6=SCL, PB7=SDA), verified Stage 4a driver, blocking
// helpers kept only for init/SWRST -- the read itself is non-blocking,
// see ImuReader below. ---

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

// Non-blocking multi-byte I2C1 read state machine. step() checks at most
// one hardware flag and returns immediately regardless of outcome -- call
// it once per EMG tick. Mirrors i2c_mpu6050_hello_main.c's verified
// mpu6050_read_regs() protocol sequence exactly, just spread across many
// calls instead of busy-waiting inside one.
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
    // start_tick: the caller's EMG-tick counter at the moment this read
    // began -- used only for the timeout below. step() itself is called
    // far more often than once per EMG tick (see main()'s loop, which
    // polls I2C as fast as the CPU can spin between ADC conversions, not
    // gated to the 1kHz tick), so counting *calls* to step() would make
    // the timeout fire in a fraction of a millisecond instead of the
    // intended ~50ms -- comparing against real elapsed EMG ticks instead
    // keeps the timeout meaning what it says regardless of poll rate.
    void begin(uint32_t start_tick) {
        state_ = ImuReadState::WaitStart1;
        start_tick_ = start_tick;
        byte_index_ = 0;
        I2C1->CR1 |= I2C_CR1_START;
    }

    bool is_idle() const { return state_ == ImuReadState::Idle; }
    const uint8_t *buf() const { return buf_; }
    int state_as_int() const { return (int)state_; } // diagnostics only

    // current_tick: the caller's current EMG-tick counter, for the same
    // timeout reason as begin() above.
    // Returns true exactly on the call a full kImuReadLen-byte reading
    // completes (buf() then holds it, valid until the next begin()).
    bool step(uint32_t current_tick) {
        if (state_ == ImuReadState::Idle) {
            return false;
        }
        if (current_tick - start_tick_ > kImuMaxTicksPerRead) {
            g_imu_state_at_timeout = (int)state_;
            g_sr1_at_timeout = I2C1->SR1;
            g_sr2_at_timeout = I2C1->SR2;
            g_timeout_count = g_timeout_count + 1;
            // A plain STOP isn't enough to recover a genuinely wedged
            // peripheral state (BUSY latched from an interrupted prior
            // transaction, not just bus contention) -- confirmed the hard
            // way in Stage 4a/4b, where only SWRST reliably cleared it.
            // Doing the same full reset+reconfigure here means a
            // transient stuck-bus condition self-recovers on the next
            // begin() instead of requiring a manual reflash.
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->CR1 |= I2C_CR1_SWRST;
            I2C1->CR1 &= ~I2C_CR1_SWRST;
            I2C1->CR1 &= ~I2C_CR1_PE;
            I2C1->CR2 = 16u;
            I2C1->CCR = 0x50u;
            I2C1->TRISE = 0x11u;
            I2C1->CR1 |= I2C_CR1_PE;
            state_ = ImuReadState::Idle;
            return false;
        }

        switch (state_) {
        case ImuReadState::Idle:
            return false;

        case ImuReadState::WaitStart1:
            if (I2C1->SR1 & I2C_SR1_SB) {
                I2C1->DR = (uint8_t)(kImuTargetAddr << 1); // address + W
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
            }
            return false;

        case ImuReadState::WaitRegTxe:
            if (I2C1->SR1 & I2C_SR1_TXE) {
                I2C1->DR = kImuRegAddr;
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
                I2C1->DR = (uint8_t)((kImuTargetAddr << 1) | 1u); // address + R
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
            }
            return false;

        case ImuReadState::ReadingMiddle:
            if (I2C1->SR1 & I2C_SR1_RXNE) {
                buf_[byte_index_] = (uint8_t)I2C1->DR;
                ++byte_index_;
                if (byte_index_ == kImuReadLen - 2u) {
                    state_ = ImuReadState::WaitBtfPenultimate;
                }
            }
            return false;

        case ImuReadState::WaitBtfPenultimate:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                buf_[kImuReadLen - 2u] = (uint8_t)I2C1->DR;
                state_ = ImuReadState::WaitBtfLast;
            }
            return false;

        case ImuReadState::WaitBtfLast:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 |= I2C_CR1_STOP;
                buf_[kImuReadLen - 1u] = (uint8_t)I2C1->DR;
                state_ = ImuReadState::Idle;
                return true; // full reading complete
            }
            return false;
        }
        return false;
    }

private:
    ImuReadState state_{ImuReadState::Idle};
    uint32_t start_tick_{0};
    uint32_t byte_index_{0};
    uint8_t buf_[kImuReadLen]{};
};

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
    gpioa_pa0_analog();
    adc1_init_timer_triggered();
    tim2_init_1khz_trgo();
    i2c1_init();
    usart2_send_string("Stage 5b: combined EMG+IMU 1kHz loop, non-blocking I2C\r\n");

    float threshold = kFallbackThreshold;
    if constexpr (kCalibrationEnabled) {
        edgeneuro::ThresholdCalibrator<float> calibrator;
        usart2_send_string("CALIBRATE: relax, then send any byte to start sampling...\r\n");
        usart2_recv_byte();
        usart2_send_string("CALIBRATE: sampling relaxed...\r\n");
        for (uint32_t i = 0; i < kCalibrationSamples;) {
            if (ADC1->SR & ADC_SR_EOC) {
                calibrator.observe_relaxed((float)(ADC1->DR & 0xFFFu));
                ++i;
            }
        }
        usart2_send_string("CALIBRATE: now clench and hold, then send any byte to start sampling...\r\n");
        usart2_recv_byte();
        usart2_send_string("CALIBRATE: sampling contracted...\r\n");
        for (uint32_t i = 0; i < kCalibrationSamples;) {
            if (ADC1->SR & ADC_SR_EOC) {
                calibrator.observe_contracted((float)(ADC1->DR & 0xFFFu));
                ++i;
            }
        }
        if (!calibrator.is_valid()) {
            usart2_send_string("CALIBRATE FAILED\r\n");
            blink_code(9);
        }
        threshold = calibrator.threshold();
        usart2_send_string("CALIBRATE OK, threshold=");
        usart2_send_uint((uint32_t)threshold);
        usart2_send_string("\r\n");
    } else {
        usart2_send_string("CALIBRATE skipped, fallback threshold=");
        usart2_send_uint((uint32_t)threshold);
        usart2_send_string("\r\n");
    }

    edgeneuro::GripStateMachine<float> grip(threshold, kOnDuration, kOffDuration);
    edgeneuro::SlewRateLimiter<float> setpoint(kSlewRate);
    edgeneuro::ComplementaryFilter<float> filter(0.98f, kDtPerTick);
    ImuReader imu_reader;
    // Trajectory smoothing (see header comment for kSmoothAlpha): y[n] =
    // alpha*x[n] + (1-alpha)*y[n-1], expressed as IirFilter's biquad.
    edgeneuro::IirFilter<float> roll_smoother(kSmoothAlpha, 0.0f, 0.0f, kSmoothAlpha - 1.0f, 0.0f);
    edgeneuro::IirFilter<float> pitch_smoother(kSmoothAlpha, 0.0f, 0.0f, kSmoothAlpha - 1.0f, 0.0f);

    uint32_t tick_count = 0;
    uint32_t imu_completions = 0;
    uint32_t last_imu_completion_tick = 0;
    uint32_t emg_window_min = 0xFFFu;
    uint32_t emg_window_max = 0u;

    float sp = 0.0f; // last EMG setpoint, for the periodic report below (updated only on EOC)
    float roll_smoothed = 0.0f;
    float pitch_smoothed = 0.0f;

    while (1) {
        // --- IMU: advance the non-blocking read every pass of this loop,
        // not just once per EMG tick -- the CPU is otherwise idle between
        // ADC conversions (up to ~1ms at 16MHz = thousands of spare
        // cycles), and polling I2C only once per tick was the actual
        // bottleneck limiting completions to ~48/s against a bus that can
        // do ~500-650/s (see PRD.md Stage 5b). Decoupling this from the
        // EOC gate lets I2C progress as fast as the hardware allows.
        if (imu_reader.is_idle()) {
            imu_reader.begin(tick_count);
        }
        if (imu_reader.step(tick_count)) {
            ++imu_completions;
            const uint8_t *b = imu_reader.buf();
            const float ax = (float)be16(&b[0]) / 16384.0f;
            const float ay = (float)be16(&b[2]) / 16384.0f;
            const float az = (float)be16(&b[4]) / 16384.0f;
            const float gx = (float)be16(&b[8]) / 131.0f * (3.14159265f / 180.0f);
            const float gy = (float)be16(&b[10]) / 131.0f * (3.14159265f / 180.0f);

            const float dt = (float)(tick_count - last_imu_completion_tick) * kDtPerTick;
            last_imu_completion_tick = tick_count;
            filter.update(gx, gy, ax, ay, az, dt); // real elapsed dt, not the fixed constructor value
            roll_smoothed = roll_smoother.process(filter.roll());
            pitch_smoothed = pitch_smoother.process(filter.pitch());
        }

        if (ADC1->SR & ADC_SR_EOC) {
            ++tick_count;

            // --- EMG: unchanged from Stage 5a ---
            const uint32_t raw = ADC1->DR & 0xFFFu;
            if (raw < emg_window_min) emg_window_min = raw;
            if (raw > emg_window_max) emg_window_max = raw;

            const bool edge = grip.update((float)raw, kDtPerTick);
            sp = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDtPerTick);
            if (edge) {
                usart2_send_string(grip.is_gripping() ? "EDGE -> Gripping\r\n" : "EDGE -> Released\r\n");
            }

            if (tick_count % 1000u == 0u) {
                usart2_send_string("tick=");
                usart2_send_uint(tick_count);
                usart2_send_string(" emg_min=");
                usart2_send_uint(emg_window_min);
                usart2_send_string(" emg_max=");
                usart2_send_uint(emg_window_max);
                usart2_send_string(" gripping=");
                usart2_send_uint(grip.is_gripping() ? 1u : 0u);
                usart2_send_string(" setpoint_x1000=");
                usart2_send_float_x1000(sp);
                usart2_send_string(" imu_completions=");
                usart2_send_uint(imu_completions);
                usart2_send_string(" imu_state=");
                usart2_send_uint((uint32_t)imu_reader.state_as_int());
                usart2_send_string(" roll_x1000=");
                usart2_send_float_x1000(filter.roll());
                usart2_send_string(" pitch_x1000=");
                usart2_send_float_x1000(filter.pitch());
                usart2_send_string(" roll_smoothed_x1000=");
                usart2_send_float_x1000(roll_smoothed);
                usart2_send_string(" pitch_smoothed_x1000=");
                usart2_send_float_x1000(pitch_smoothed);
                usart2_send_string("\r\n");
                GPIOC->ODR ^= (1u << LED_PIN);
                emg_window_min = 0xFFFu;
                emg_window_max = 0u;
                imu_completions = 0;
            }
        }
    }
}

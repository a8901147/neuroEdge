// Stage 5a: the EMG half of Phase 3's split control loop (see PRD.md
// Section 3's Phase 3 control-architecture note), running on real
// hardware for the first time -- GripStateMachine and SlewRateLimiter
// were only ever exercised against synthetic values in Host tests
// (tests/test_grip_state_machine.cpp, tests/test_slew_rate_limiter.cpp)
// until now. Deliberately independent of the MPU6050/IMU side (currently
// blocked -- see Stage 4b), and of Pipeline/EdgeNeuro<> entirely, per the
// same architecture decision.
//
// Timer + ADC setup is copied verbatim from Stage 3c/3d
// (timer_adc_1khz_main.c) -- TIM2 TRGO -> ADC1 EXTSEL hardware trigger,
// already empirically confirmed accurate to <0.5% (PRD.md Stage 3c/3d).
// That real, measured 1ms period is why dt=0.001f below is a fact, not a
// guess, unlike Stage 4b's I2C loop where the assumed dt turned out to be
// off by 35% (see complementary_filter_stress_test_main.cpp).
//
// Threshold can be computed at boot by ThresholdCalibrator
// (include/edgeneuro/control/threshold_calibrator.hpp) from a short
// relax-then-contract sequence, since a fixed magic number only holds for
// one exact electrode placement, skin contact, and gain-pot setting. BUT
// that sequence blocks main() waiting for a UART trigger byte per phase
// (see the calibration section below) -- if nothing ever sends one (board
// power-cycled with no host attached, or a host attaches after boot and
// doesn't know to send bytes), the board sits silently forever, never
// reaching the real control loop. Found this out directly: a mid-session
// SWD/reset event left the board silently stuck at the first
// usart2_recv_byte() call with zero UART output, which looked identical
// to "nothing is working" from the outside.
//
// kCalibrationEnabled defaults OFF so a normal power-up always reaches
// the control loop immediately, using kFallbackThreshold (the last known-
// good value from an actual calibration run, see PRD.md Stage 5a) instead
// of blocking on human/host interaction. Flip it to true, reflash, and
// recalibrate whenever electrode placement or skin contact actually
// changes enough to matter -- not on every boot.
static constexpr bool kCalibrationEnabled = false;
static constexpr float kFallbackThreshold = 2037.0f; // from the last successful CALIBRATE OK

#include <cstdint>

#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/slew_rate_limiter.hpp"
#include "edgeneuro/control/threshold_calibrator.hpp"
#include "stm32f4xx.h"

#define LED_PIN 13u

static constexpr float kOnDuration = 0.15f;  // seconds of sustained above-threshold to grip
static constexpr float kOffDuration = 0.15f; // seconds of sustained below-threshold to release
static constexpr float kSlewRate = 5.0f;     // setpoint units/sec -- 1/5=0.2s full-stroke ramp
static constexpr float kDt = 0.001f;         // TIM2-verified exact 1kHz, see header comment
static constexpr uint32_t kCalibrationSamples = 3000u; // ~3s per phase @ 1kHz

static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

static void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));

    USART2->BRR = 0x0683u;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; // RE added: calibration waits on RX
}

// Blocks until any byte arrives -- used to let a human/host trigger each
// calibration phase at exactly the right moment (see main()), instead of
// guessing a fixed delay that can't stay in sync with someone reacting to
// printed instructions over a serial link with real round-trip latency.
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

// Prints a float as an integer x1000 (e.g. 0.734 -> "734") -- same
// no-float-formatting workaround as complementary_filter_hello_main.cpp.
static void usart2_send_float_x1000(float value) {
    const int32_t thousandths = (int32_t)(value * 1000.0f);
    if (thousandths < 0) {
        usart2_send_byte('-');
        usart2_send_uint((uint32_t)(-thousandths));
    } else {
        usart2_send_uint((uint32_t)thousandths);
    }
}

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

// Blinks `code` short pulses then a long pause, forever -- UART-independent
// diagnostic, same pattern as every other stage's blink_code().
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
    usart2_send_string("Stage 5a: EMG grip control, real MyoWare on PA0\r\n");

    // Calibration is opt-in (kCalibrationEnabled, see header comment) --
    // when off, skip straight to kFallbackThreshold so a normal power-up
    // always reaches the control loop without needing anyone to send a
    // UART trigger byte.
    float threshold = kFallbackThreshold;

    if constexpr (kCalibrationEnabled) {
        // Two phases, blocking the main loop on purpose -- this only runs
        // once at boot, not in the hot path. Each phase waits for an
        // arbitrary RX byte before it starts sampling, instead of a fixed
        // delay: a blind timer can't stay synchronized with a human
        // reacting to printed instructions over a serial link with real
        // round-trip latency (confirmed the hard way -- the first two
        // attempts at a fixed 2s/3s delay both ran the sampling window
        // before anyone was actually relaxed/clenching, and silently
        // failed). Whatever sends the trigger byte (a human pressing
        // Enter in a terminal, or a host script) decides exactly when
        // each phase starts.
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

        usart2_send_string("CALIBRATE: relaxed_max=");
        usart2_send_uint((uint32_t)calibrator.relaxed_max());
        usart2_send_string(" contracted_min=");
        usart2_send_uint((uint32_t)calibrator.contracted_min());
        usart2_send_string("\r\n");

        if (!calibrator.is_valid()) {
            usart2_send_string("CALIBRATE FAILED -- no clean separation, check electrodes\r\n");
            blink_code(9);
        }

        threshold = calibrator.threshold();
        usart2_send_string("CALIBRATE OK, threshold=");
        usart2_send_uint((uint32_t)threshold);
        usart2_send_string("\r\n");
    } else {
        usart2_send_string("CALIBRATE skipped (kCalibrationEnabled=false), using fallback threshold=");
        usart2_send_uint((uint32_t)threshold);
        usart2_send_string("\r\n");
    }

    edgeneuro::GripStateMachine<float> grip(threshold, kOnDuration, kOffDuration);
    edgeneuro::SlewRateLimiter<float> setpoint(kSlewRate);

    uint32_t sample_count = 0;
    uint32_t window_min = 0xFFFu;
    uint32_t window_max = 0u;

    while (1) {
        if (ADC1->SR & ADC_SR_EOC) {
            const uint32_t raw = ADC1->DR & 0xFFFu;
            if (raw < window_min) window_min = raw;
            if (raw > window_max) window_max = raw;
            ++sample_count;

            const bool edge = grip.update((float)raw, kDt);
            const float sp = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDt);

            if (edge) { // print immediately on transition, not just periodically
                usart2_send_string(grip.is_gripping() ? "EDGE -> Gripping\r\n" : "EDGE -> Released\r\n");
            }

            // Periodic summary, same 1000-sample (~1s) cadence as Stage
            // 3c/3d -- 9600 baud can't sustain one line per 1ms sample.
            if (sample_count % 1000u == 0u) {
                usart2_send_string("raw_min=");
                usart2_send_uint(window_min);
                usart2_send_string(" raw_max=");
                usart2_send_uint(window_max);
                usart2_send_string(" gripping=");
                usart2_send_uint(grip.is_gripping() ? 1u : 0u);
                usart2_send_string(" setpoint_x1000=");
                usart2_send_float_x1000(sp);
                usart2_send_string("\r\n");
                GPIOC->ODR ^= (1u << LED_PIN);
                window_min = 0xFFFu;
                window_max = 0u;
            }
        }
    }
}

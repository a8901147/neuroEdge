// Stage 7d (Phase 4 bring-up): 4-channel follow-up to servo_limit_finder
// -- once all 4 servos are actually mounted on the assembled MEArm, each
// joint's REAL usable range is capped by the linkage geometry, not just
// the bare servo's own mechanical limit (see SESSION_LOG's 2026-09-21
// entry: the single already-tested SG92R measured a 400-2550us range
// bare, but community MeArm calibration data shows the assembled linkage
// only allows roughly 90-100 degrees of real joint travel regardless).
// This tool lets each of the 4 mounted servos be limit-found in one
// session instead of reflashing servo_limit_finder 4 times with the pin
// hardcoded differently each time.
//
// Protocol over the same USART2 link as servo_limit_finder (115200 baud):
//   '1'/'2'/'3'/'4' -- select the active channel (base/shoulder/elbow/
//                      claw, same bottom-to-top assignment as
//                      servo_pwm_4ch_test_main.c -- see that file's header
//                      comment for why that's the physical order).
//   '+'/'-'         -- nudge the CURRENTLY SELECTED channel's pulse width,
//                      same one-byte-per-nudge behavior as
//                      servo_limit_finder_main.c.
// Switching channels does NOT move the newly-selected channel back to
// center -- it stays wherever it was last left, so switching to check on
// a different joint doesn't disturb one you're mid-calibration on.
//
// PWM/GPIO setup (TIM3 all 4 channels, PA6/PA7/PB0/PB1) is byte-for-byte
// servo_pwm_4ch_test_main.c's -- see that file's header comment for the
// full RM0368/CMSIS-header register verification, not repeated here.
// USART2 setup is byte-for-byte servo_limit_finder_main.c's (PA2=TX,
// PA3=RX, AF7, 115200 baud), already independently verified there.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13, toggled once per accepted command

static void led_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u));
}

static void led_toggle(void) {
    GPIOC->ODR ^= (1u << LED_PIN);
}

static void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));

    USART2->BRR = 0x008Bu; // 115200 baud @ 16MHz HSI, OVER8=0 -- value and RM0368 19.3.4
                           // derivation are phase3_control_loop_main.cpp's (~0.08% error)
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
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

static int usart2_try_read_byte(uint8_t *out) {
    if (USART2->SR & USART_SR_RXNE) {
        *out = (uint8_t)USART2->DR;
        return 1;
    }
    return 0;
}

// PA6 (TIM3_CH1) and PA7 (TIM3_CH2) -- see servo_pwm_4ch_test_main.c.
static void gpioa_tim3_ch1_ch2_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOA->MODER |= (2u << (6u * 2u)) | (2u << (7u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 6u)) | (0xFu << (4u * 7u)));
    GPIOA->AFR[0] |= (2u << (4u * 6u)) | (2u << (4u * 7u));
}

// PB0 (TIM3_CH3) and PB1 (TIM3_CH4) -- see servo_pwm_4ch_test_main.c.
static void gpiob_tim3_ch3_ch4_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3u << (0u * 2u)) | (3u << (1u * 2u)));
    GPIOB->MODER |= (2u << (0u * 2u)) | (2u << (1u * 2u));
    GPIOB->AFR[0] &= ~((0xFu << (4u * 0u)) | (0xFu << (4u * 1u)));
    GPIOB->AFR[0] |= (2u << (4u * 0u)) | (2u << (4u * 1u));
}

// 2026-09-26: every channel boots at the arm's chosen REST pose -- base/shoulder/elbow
// 1500us, claw 1300us (open) -- the same pulses phase3_control_loop uses
// (include/edgeneuro/control/mearm_servo_maps.hpp k*RestUs; a test keeps the two equal).
// This used to be a common 1475us (the bare servo's measured centre), so every pose /
// linkage-measurement session began from a different pose than the arm's chosen start.
// The very first pulse after power-up still moves each servo from wherever it was, at
// its own full speed: clear the space around the arm before powering up.
// Channel order: 1=base, 2=shoulder, 3=elbow, 4=claw.
static const uint32_t REST_PULSE_US[4] = {1500u, 1500u, 1500u, 1300u};

static void tim3_pwm_50hz_4ch_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 15u;
    TIM3->ARR = 19999u;

    TIM3->CCR1 = REST_PULSE_US[0];
    TIM3->CCR2 = REST_PULSE_US[1];
    TIM3->CCR3 = REST_PULSE_US[2];
    TIM3->CCR4 = REST_PULSE_US[3];

    TIM3->CCMR1 = (TIM3->CCMR1 & ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M)) |
                  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) |
                  (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM3->CCMR2 = (TIM3->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M)) |
                  (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) |
                  (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CR1 |= TIM_CR1_CEN;
}

static volatile uint32_t *const CHANNEL_CCR[4] = {
    &TIM3->CCR1, // 1 = base
    &TIM3->CCR2, // 2 = shoulder
    &TIM3->CCR3, // 3 = elbow
    &TIM3->CCR4, // 4 = claw
};
static const char *const CHANNEL_NAME[4] = {"base", "shoulder", "elbow", "claw"};

// Same software sanity bounds as servo_limit_finder_main.c -- NOT the
// servo's real mechanical limit, just wide/tight enough that this tool
// itself can't be the thing that commands something absurd.
#define PULSE_US_FLOOR 300u
#define PULSE_US_CEIL 2700u
#define PULSE_US_STEP 25u

static void report_state(int active_ch, uint32_t pulse_us[4]) {
    usart2_send_string("channel=");
    usart2_send_string(CHANNEL_NAME[active_ch]);
    usart2_send_string(" pulse_us=");
    usart2_send_uint(pulse_us[active_ch]);
    usart2_send_string("\n");
}

int main(void) {
    led_init();
    usart2_init();
    gpioa_tim3_ch1_ch2_af();
    gpiob_tim3_ch3_ch4_af();
    tim3_pwm_50hz_4ch_init();

    uint32_t pulse_us[4] = {
        REST_PULSE_US[0], REST_PULSE_US[1],
        REST_PULSE_US[2], REST_PULSE_US[3],
    };
    int active_ch = 0; // starts on "base" (channel 1)

    usart2_send_string("servo_limit_finder_4ch ready. send '1'-'4' to select "
                        "a channel (base/shoulder/elbow/claw), '+'/'-' to "
                        "nudge it.\n");
    report_state(active_ch, pulse_us);

    for (;;) {
        uint8_t b;
        if (!usart2_try_read_byte(&b)) {
            continue;
        }

        if (b >= '1' && b <= '4') {
            active_ch = (int)(b - '1');
            led_toggle();
            report_state(active_ch, pulse_us);
            continue;
        }

        if (b == '+' && pulse_us[active_ch] + PULSE_US_STEP <= PULSE_US_CEIL) {
            pulse_us[active_ch] += PULSE_US_STEP;
        } else if (b == '-' && pulse_us[active_ch] >= PULSE_US_FLOOR + PULSE_US_STEP) {
            pulse_us[active_ch] -= PULSE_US_STEP;
        } else {
            continue; // ignore anything else, including a clamped +/-
        }

        *CHANNEL_CCR[active_ch] = pulse_us[active_ch];
        led_toggle();
        report_state(active_ch, pulse_us);
    }
}

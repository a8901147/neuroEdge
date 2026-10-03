// Stage 7c (Phase 4 bring-up): extends servo_pwm_test_main.c's single-
// channel PA6/TIM3_CH1 PWM to all 4 of TIM3's channels, one per MEArm
// servo. Same TIM3 (PSC=15/ARR=19999 -> exactly 50Hz, 1 tick = 1us) drives
// all 4 channels simultaneously -- they share one counter, only each
// channel's own CCRx (pulse width) is independent.
//
// Channel <-> physical servo assignment, per explicit instruction
// (2026-09-21): assign in increasing channel order from the MEArm's
// bottom-most servo to its top-most (base -> shoulder -> elbow -> claw is
// MEArm's real physical stacking order -- the base servo sits at the very
// bottom and rotates the whole arm, the claw servo sits at the very top/
// far end as the end effector; there is no left/right ambiguity to break
// since each height stage is a single servo, not a pair).
//   TIM3_CH1 (PA6) = base   (bottom-most)
//   TIM3_CH2 (PA7) = shoulder
//   TIM3_CH3 (PB0) = elbow
//   TIM3_CH4 (PB1) = claw   (top-most / end effector)
//
// Pin choice reasoning (same as servo_pwm_test_main.c): PA0 is EMG ADC,
// PA2/PA3 are USART2, PB6/PB7 are I2C1 (both IMUs), PC13 is the LED -- all
// already spoken for by phase3_control_loop_main.cpp. TIM3's own 4
// channels (PA6/PA7/PB0/PB1) are the only pins on this board that give 4
// independent PWM outputs from a single timer without touching any of
// those, confirmed free.
//
// Register values verified the same way as servo_pwm_test_main.c (RM0368
// Rev 5 + the vendored CMSIS header, firmware/build/_deps/cmsis_device_f4-
// src) -- see that file's header comment for the full TIM3/PWM-mode/AF2
// verification, not repeated here. New for this file, confirmed present
// in the same vendored stm32f401xc.h before use (not assumed):
//   - RCC_AHB1ENR_GPIOBEN (needed for PB0/PB1, already relied on
//     elsewhere for PB6/PB7's I2C1 pins in phase3_control_loop_main.cpp).
//   - TIM_CCMR1_OC2M/OC2M_1/OC2M_2 (bits[14:12], CH2 lives in the same
//     CCMR1 register as CH1), TIM_CCMR2_OC3M/OC4M (bits[6:4]/[14:12], CH3/
//     CH4 live in CCMR2 -- a SEPARATE register from CCMR1, RM0368 13.4.8).
//   - TIM_CCER_CC2E/CC3E/CC4E (bits 4/8/12 respectively, RM0368 13.4.9 --
//     same register as CH1's CC1E, just one nibble per channel).
// PA7/PB0/PB1 -> AF2 for TIM3_CH2/3/4: same AF2-for-TIM3-family reasoning
// and same caveat as servo_pwm_test_main.c's PA6 (not independently
// verified against the raw datasheet Table 9 PDF -- that fetch timed out
// -- verified instead via ST's own GPIO_AF2_TIM3 HAL macro plus an
// independent pin-table source). If any ONE of these 4 pins doesn't move
// its servo when flashed, this AF number is the first thing to re-check.
//
// Bring-up behavior: NOT a simultaneous 4-servo sweep (deliberately, for
// this first test) -- moves ONE channel at a time, in the
// base->shoulder->elbow->claw order above, holding at center between each,
// so whichever specific channel/pin is miswired shows up as "this one
// servo didn't move during its turn" rather than 4 servos all moving at
// once with no way to tell which one (if any) is actually wired wrong.
// Uses each servo's SHARED (not yet per-unit-calibrated) real range from
// servo_limit_finder's first measurement (center=1475, safe 450-2500) --
// per-channel calibration once each servo is actually mounted on the
// assembled MEArm is a separate, later step (the linkage geometry caps
// each joint's real usable range well before this raw servo range, see
// SESSION_LOG's 2026-09-21 entry), not assumed identical across all 4
// units here.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13, toggled once per channel moved (heartbeat)

// Same real-measured-limits reasoning as servo_pwm_test_main.c's
// 2026-09-21 update -- applied here as a shared STARTING POINT for all 4
// channels until each servo gets its own real per-unit/per-joint
// calibration (see this file's header comment).
#define SERVO_CENTER_PULSE_US 1475u
#define SERVO_PULSE_MIN_US 450u
#define SERVO_PULSE_MAX_US 2500u

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

static void led_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u));
}

static void led_toggle(void) {
    GPIOC->ODR ^= (1u << LED_PIN);
}

// PA6 (TIM3_CH1) and PA7 (TIM3_CH2) -- both AFR[0] (pins 0-7), AF2.
static void gpioa_tim3_ch1_ch2_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOA->MODER |= (2u << (6u * 2u)) | (2u << (7u * 2u)); // AF mode (10)

    GPIOA->AFR[0] &= ~((0xFu << (4u * 6u)) | (0xFu << (4u * 7u)));
    GPIOA->AFR[0] |= (2u << (4u * 6u)) | (2u << (4u * 7u)); // AF2 = TIM3
}

// PB0 (TIM3_CH3) and PB1 (TIM3_CH4) -- both AFR[0] (pins 0-7), AF2.
static void gpiob_tim3_ch3_ch4_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER &= ~((3u << (0u * 2u)) | (3u << (1u * 2u)));
    GPIOB->MODER |= (2u << (0u * 2u)) | (2u << (1u * 2u));

    GPIOB->AFR[0] &= ~((0xFu << (4u * 0u)) | (0xFu << (4u * 1u)));
    GPIOB->AFR[0] |= (2u << (4u * 0u)) | (2u << (4u * 1u));
}

static void tim3_pwm_50hz_4ch_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 15u;    // TIM3CLK/16 = 16MHz/16 = 1MHz -> 1 tick = 1us
    TIM3->ARR = 19999u; // 20000 ticks x 1us = 20ms period -> exactly 50Hz

    TIM3->CCR1 = SERVO_CENTER_PULSE_US; // base
    TIM3->CCR2 = SERVO_CENTER_PULSE_US; // shoulder
    TIM3->CCR3 = SERVO_CENTER_PULSE_US; // elbow
    TIM3->CCR4 = SERVO_CENTER_PULSE_US; // claw

    // CH1/CH2 -> CCMR1, CH3/CH4 -> CCMR2 (RM0368 13.4.7/13.4.8 -- two
    // channels per register, NOT all 4 in one like CCER below).
    TIM3->CCMR1 = (TIM3->CCMR1 & ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M)) |
                  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | // CH1: PWM mode 1
                  (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);  // CH2: PWM mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    TIM3->CCMR2 = (TIM3->CCMR2 & ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M)) |
                  (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | // CH3: PWM mode 1
                  (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);  // CH4: PWM mode 1
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    // All 4 channels' enable bits live together in CCER (RM0368 13.4.9).
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CR1 |= TIM_CR1_CEN;
}

// One entry per servo, in the required bottom-to-top physical order --
// see this file's header comment for why base->shoulder->elbow->claw is
// that order, not an arbitrary channel-number pick.
static volatile uint32_t *const CHANNEL_CCR[4] = {
    &TIM3->CCR1, // base
    &TIM3->CCR2, // shoulder
    &TIM3->CCR3, // elbow
    &TIM3->CCR4, // claw
};

int main(void) {
    led_init();
    gpioa_tim3_ch1_ch2_af();
    gpiob_tim3_ch3_ch4_af();
    tim3_pwm_50hz_4ch_init();

    for (;;) {
        for (int ch = 0; ch < 4; ++ch) {
            *CHANNEL_CCR[ch] = SERVO_PULSE_MIN_US;
            led_toggle();
            delay(1500000u);

            *CHANNEL_CCR[ch] = SERVO_PULSE_MAX_US;
            led_toggle();
            delay(1500000u);

            *CHANNEL_CCR[ch] = SERVO_CENTER_PULSE_US; // back to center before
            led_toggle();                             // moving on to the
            delay(800000u);                           // next channel
        }
    }
}

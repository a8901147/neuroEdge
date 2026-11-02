// MeArm servo outputs: TIM3 CH1..CH4 at 50 Hz on PA6 (base), PA7 (shoulder), PB0 (elbow), PB1 (claw), AF2.
// Included only by phase3_control_loop_main.cpp, and only with EDGENEURO_DRIVE_SERVOS (firmware/CMakeLists.txt
// explains why the default build leaves TIM3 and these pins untouched). Register values are the ones verified in
// servo_pwm_4ch_test_main.c.
#pragma once

#include "edgeneuro/control/mearm_servo_maps.hpp"
#include "stm32f4xx.h"

static void gpioa_tim3_ch1_ch2_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOA->MODER |= (2u << (6u * 2u)) | (2u << (7u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 6u)) | (0xFu << (4u * 7u)));
    GPIOA->AFR[0] |= (2u << (4u * 6u)) | (2u << (4u * 7u)); // AF2 = TIM3
}

static void gpiob_tim3_ch3_ch4_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~((3u << (0u * 2u)) | (3u << (1u * 2u)));
    GPIOB->MODER |= (2u << (0u * 2u)) | (2u << (1u * 2u));
    GPIOB->AFR[0] &= ~((0xFu << (4u * 0u)) | (0xFu << (4u * 1u)));
    GPIOB->AFR[0] |= (2u << (4u * 0u)) | (2u << (4u * 1u));
}

// The servo block in main() runs on every 10th 1 kHz tick.
static constexpr float kServoStepDt = 10.0f * 0.001f;

// 1 MHz timer clock (PSC 15), 20 ms period (ARR 19999), PWM mode 1 with preload, so CCRx is the pulse width in us
// and a new value takes effect at the next 50 Hz frame. Every channel starts at its rest pulse (mearm_servo_maps.hpp).
// The very first pulse after power-up moves each servo at its own full speed; after that, main() walks each channel
// to its target through a ServoStartupRamp.
static void tim3_pwm_50hz_4ch_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 15u;
    TIM3->ARR = 19999u;

    TIM3->CCR1 = edgeneuro::mearm::kBaseRestUs;
    TIM3->CCR2 = edgeneuro::mearm::kShoulderRestUs;
    TIM3->CCR3 = edgeneuro::mearm::kElbowRestUs;
    TIM3->CCR4 = edgeneuro::mearm::kClawRestUs;

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

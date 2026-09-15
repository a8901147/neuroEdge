// Stage 7a (Phase 4 bring-up): generates 50Hz hobby-servo PWM on PA6
// (TIM3_CH1) and sweeps the pulse width between 1ms and 2ms so a single
// SG90 wired up as a standalone bring-up test can be visually confirmed to
// sweep end-to-end, before any of this is wired into the real Phase 3
// control loop. PA6 was picked because it is free: PA0 is EMG ADC,
// PA2/PA3 are USART2, PB6/PB7 are I2C1 (both IMUs), PC13 is the LED --
// see phase3_control_loop_main.cpp. TIM3 (not TIM2) was picked specifically
// so this doesn't collide with TIM2's existing 1kHz ADC-trigger TRGO
// config from Stage 3c/3d.
//
// Register values verified against the official STM32F401 reference
// manual (RM0368 Rev 5) and the CMSIS device header already used by every
// prior stage (firmware/build/_deps/cmsis_device_f4-src), not guessed:
//   - RCC_APB1ENR bit 1 = TIM3EN (confirmed directly from the vendored
//     stm32f401xc.h's RCC_APB1ENR_TIM3EN_Pos = 1U, same register as
//     Stage 3c/3d's TIM2EN at bit 0).
//   - TIM3CLK = HCLK = 16MHz, same reasoning as Stage 3c/3d (APB1
//     prescaler is at its reset value of 1, so no x2 multiplier applies).
//   - PWM period = (PSC+1) x (ARR+1) / TIM3CLK (RM0368 13.3.1, same
//     formula Stage 3c/3d used for its 1kHz ADC trigger). For an exact
//     50Hz (20ms) period with 1us-resolution pulse widths: PSC=15
//     (PSC+1=16 -> 16MHz/16 = 1MHz, 1 tick = 1us), ARR=19999
//     (ARR+1=20000 ticks x 1us = 20ms = 50Hz exactly).
//   - TIMx_CCMR1 bits[6:4] = OC1M, 110 = "PWM mode 1" (RM0368 13.4.7):
//     OC1REF is high while CNT < CCR1, low otherwise -- CCR1 in
//     microseconds (thanks to the 1MHz timer clock above) directly IS the
//     servo pulse width. TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1 confirmed as
//     bits 6/5 (OC1M_Pos=4) in the vendored header.
//   - TIMx_CCMR1 bit 3 = OC1PE (output compare 1 preload enable, RM0368
//     13.4.7) -- buffers CCR1 writes until the next update event so a
//     pulse-width change mid-sweep can never glitch a single already-
//     in-progress pulse.
//   - TIMx_CR1 bit 7 = ARPE (auto-reload preload enable, RM0368 13.4.1),
//     paired with OC1PE for the same glitch-free-update reasoning.
//   - TIMx_CCER bit 0 = CC1E, capture/compare 1 output enable (RM0368
//     13.4.9) -- routes OC1REF out to the actual PA6 pin.
//   - TIMx_CR1 bit 0 = CEN, counter enable (RM0368 13.4.1), same bit
//     Stage 3c/3d already relies on for TIM2.
//   - PA6 = TIM3_CH1 via AF2: NOT verified against the raw datasheet
//     Table 9 PDF directly (that fetch timed out both times it was
//     tried) -- verified instead via ST's own published HAL macro
//     GPIO_AF2_TIM3 == 0x02 (ST's stm32f4xx_hal_gpio_ex.h, same AF
//     numbering scheme used across the whole F4 family: TIM1/TIM2=AF1,
//     TIM3/4/5=AF2) plus an independent pin-table source confirming PA6
//     genuinely carries TIM3_CH1. Flagged here explicitly rather than
//     presented as RM0368-certain, per this project's own "don't claim
//     more certainty than what was actually checked" convention -- if
//     this pin doesn't move the servo when flashed, this AF number is the
//     first thing to re-verify against the real datasheet PDF.
//
// Wiring: SG90 brown/black=GND (common ground with the board), red=+5V
// (the Black Pill's 5V/VBUS pin, NOT 3V3 -- a servo needs 5V), orange or
// yellow=signal to PA6.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13, toggled once per sweep step as a heartbeat

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

static void led_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u)); // general-purpose output
}

static void led_toggle(void) {
    GPIOC->ODR ^= (1u << LED_PIN);
}

static void gpioa_pa6_tim3_ch1_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (6u * 2u));
    GPIOA->MODER |= (2u << (6u * 2u)); // PA6 -> alternate function mode (10)
    GPIOA->AFR[0] &= ~(0xFu << (4u * 6u));
    GPIOA->AFR[0] |= (2u << (4u * 6u)); // AF2 = TIM3_CH1, see header comment
}

static void tim3_pwm_50hz_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 15u;    // TIM3CLK/16 = 16MHz/16 = 1MHz -> 1 tick = 1us
    TIM3->ARR = 19999u; // 20000 ticks x 1us = 20ms period -> exactly 50Hz
    TIM3->CCR1 = 1500u; // start centered, ~1.5ms pulse (servo mid-travel)

    TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M) |
                  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // 110 = PWM mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // preload CCR1 writes
    TIM3->CCER |= TIM_CCER_CC1E;    // route OC1REF out to the PA6 pin
    TIM3->CR1 |= TIM_CR1_ARPE;      // preload ARR writes (paired with OC1PE)
    TIM3->CR1 |= TIM_CR1_CEN;       // start counting -- PWM begins here
}

// Standard hobby-servo pulse convention: ~1000us = one end of travel,
// ~1500us = center, ~2000us = the other end. Some servos (SG90 included)
// tolerate a bit more than this before hitting their mechanical stop --
// staying inside 1000-2000 here deliberately, so this bring-up test can't
// itself stall the servo against its own end-stop.
#define SERVO_PULSE_MIN_US 1000u
#define SERVO_PULSE_MAX_US 2000u
#define SERVO_PULSE_STEP_US 20u

int main(void) {
    led_init();
    gpioa_pa6_tim3_ch1_af();
    tim3_pwm_50hz_init();

    uint32_t pulse_us = SERVO_PULSE_MIN_US;
    int direction = 1;

    for (;;) {
        TIM3->CCR1 = pulse_us;
        led_toggle();
        delay(200000u); // rough step delay, not calibrated -- just needs
                         // to be slow enough to watch the servo sweep

        if (direction > 0) {
            if (pulse_us + SERVO_PULSE_STEP_US >= SERVO_PULSE_MAX_US) {
                pulse_us = SERVO_PULSE_MAX_US;
                direction = -1;
            } else {
                pulse_us += SERVO_PULSE_STEP_US;
            }
        } else {
            if (pulse_us <= SERVO_PULSE_MIN_US + SERVO_PULSE_STEP_US) {
                pulse_us = SERVO_PULSE_MIN_US;
                direction = 1;
            } else {
                pulse_us -= SERVO_PULSE_STEP_US;
            }
        }
    }
}

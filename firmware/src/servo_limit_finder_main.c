// Phase 4 bring-up, follow-up to servo_pwm_test: an interactive version
// for finding a specific servo's REAL safe pulse-width range by hand
// (rather than guessing a wider range and hoping) -- send '+' over UART to
// nudge the pulse width up by a small step, '-' to nudge it down, and read
// the current value back over the same UART. Stop nudging the instant the
// servo sounds like it's grinding/straining against its own mechanical
// end-stop; whatever value was last printed before that point is this
// servo's real usable limit on that side.
//
// PA6/TIM3_CH1 PWM setup identical to servo_pwm_test_main.c -- see that
// file's header comment for the full RM0368/CMSIS-header verification of
// every TIM3/GPIO register value used here, not repeated here.
//
// USART2 (PA2=TX, PA3=RX, AF7, 9600 baud) reused byte-for-byte from
// phase3_control_loop_main.cpp's already-verified usart2_init/
// usart2_send_uint/non-blocking-RXNE-poll pattern.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13, toggled once per accepted +/- command

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

    USART2->BRR = 0x0683u; // 9600 baud @ 16MHz HSI, OVER8=0
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

static void gpioa_pa6_tim3_ch1_af(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (6u * 2u));
    GPIOA->MODER |= (2u << (6u * 2u));
    GPIOA->AFR[0] &= ~(0xFu << (4u * 6u));
    GPIOA->AFR[0] |= (2u << (4u * 6u)); // AF2 = TIM3_CH1
}

static void tim3_pwm_50hz_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 15u;
    TIM3->ARR = 19999u;
    TIM3->CCR1 = 1500u;
    TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M) |
                  (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CR1 |= TIM_CR1_CEN;
}

// Software sanity bounds only -- NOT the servo's real mechanical limit,
// which is exactly what this tool exists to find by ear. Wide enough to
// not get in the way, tight enough that a stray/garbled byte can't command
// something absurd.
#define PULSE_US_FLOOR 300u
#define PULSE_US_CEIL 2700u
#define PULSE_US_STEP 25u

int main(void) {
    led_init();
    usart2_init();
    gpioa_pa6_tim3_ch1_af();
    tim3_pwm_50hz_init();

    uint32_t pulse_us = 1500u;
    usart2_send_string("servo_limit_finder ready. send '+' or '-' (one byte "
                        "each), starting pulse_us=");
    usart2_send_uint(pulse_us);
    usart2_send_string("\n");

    for (;;) {
        uint8_t b;
        if (usart2_try_read_byte(&b)) {
            if (b == '+' && pulse_us + PULSE_US_STEP <= PULSE_US_CEIL) {
                pulse_us += PULSE_US_STEP;
            } else if (b == '-' && pulse_us >= PULSE_US_FLOOR + PULSE_US_STEP) {
                pulse_us -= PULSE_US_STEP;
            } else {
                continue; // ignore anything else (including a clamped +/-)
            }
            TIM3->CCR1 = pulse_us;
            led_toggle();
            usart2_send_string("pulse_us=");
            usart2_send_uint(pulse_us);
            usart2_send_string("\n");
        }
    }
}

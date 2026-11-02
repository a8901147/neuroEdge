// EMG input: ADC1 channel 0 (PA0, MyoWare ENV output), converted on every TIM2 update (1 kHz).
// Included only by phase3_control_loop_main.cpp (one translation unit).
//
// The main loop polls ADC1 EOC, so each conversion is both the EMG sample and the loop's 1 ms tick.
#pragma once

#include "board.hpp"
#include "stm32f4xx.h"

static void gpioa_pa0_analog(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (0u * 2u));
    GPIOA->MODER |= (3u << (0u * 2u));
}

// 16 MHz / (PSC + 1) = 1 MHz, / (ARR + 1) = 1 kHz; TRGO on update (MMS = 010) triggers ADC1 (RM0368 ch. 13).
static void tim2_init_1khz_trgo(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15u;
    TIM2->ARR = 999u;
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;
    TIM2->CR1 |= TIM_CR1_CEN;
}

// One regular conversion of channel 0, started by TIM2 TRGO on its rising edge (EXTEN = 01, EXTSEL = 0110;
// RM0368 ch. 11).
static void adc1_init_timer_triggered(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3 &= ~ADC_SQR3_SQ1;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(10000u);
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL)) |
                ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
}

// Stage 0 bring-up: blink the Black Pill's onboard LED (PC13, active-low
// on this board family). Proves the whole toolchain -> flash -> execute
// path works before anything from include/edgeneuro/ gets involved.
//
// No clock configuration: runs on the default reset clock (internal HSI,
// 16MHz) since blink timing doesn't need to be precise yet. Stage 3 will
// configure the PLL for a real 84MHz + timer-driven 1kHz ADC sample clock.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u)); // general purpose output

    while (1) {
        GPIOC->ODR ^= (1u << LED_PIN);
        delay(800000u); // rough ~ half-second-ish toggle at 16MHz HSI, not calibrated
    }
}

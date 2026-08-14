// Stage 3a: USART2 polling transmit-only, PA2(TX)/PA3(RX) at 9600 8N1 on
// the default 16MHz HSI clock (no PLL/precise clock config yet -- same as
// Stage 0). Isolates ONE new subsystem (UART) before adding Timer+ADC on
// top of it, per this project's incremental-stage discipline -- if this
// doesn't work, the problem is UART wiring/config, not ADC/Timer.
//
// Every register value below is verified against the official STM32F401
// reference manual (RM0368 Rev 5) and the STM32F401CCU6 datasheet's
// Table 9 "Alternate function mapping" -- not guessed:
//   - PA2/PA3 = AF7 for USART2_TX/USART2_RX (datasheet DocID024738 Table 9)
//   - RCC_AHB1ENR bit 0 = GPIOAEN (RM0368 6.3.9)
//   - RCC_APB1ENR bit 17 = USART2EN (RM0368 6.3.11)
//   - USART_CR1: bit 15 = OVER8, bit 13 = UE, bit 12 = M (0 = 8 data bits,
//     reset default), bit 3 = TE (RM0368 19.6.4)
//   - USART_SR bit 7 = TXE, transmit data register empty (RM0368 19.6.1)
//   - Baud = f_CK / (16 x USARTDIV) when OVER8=0 (RM0368 19.3.4, Eq. 1).
//     At f_CK=16MHz (default HSI, no prescaler configured) for 9600 baud:
//     USARTDIV = 16000000 / (16*9600) = 104.1875 ->
//     DIV_Mantissa=104 (0x68), DIV_Fraction=3 (0x3) -> BRR=0x0683,
//     actual baud ~9598 (0.02% error). Chose 9600 over a higher rate
//     specifically because HSI's uncalibrated tolerance eats less margin
//     at a slower baud, and oversampling by 16 (vs. 8) is the more
//     clock-deviation-tolerant option (RM0368 19.3.3) -- both matter more
//     without a precise external crystal driving the clock.
// All register field macros (RCC_AHB1ENR_GPIOAEN, USART_CR1_UE, etc.) are
// from the FetchContent'd CMSIS device header (stm32f401xc.h), confirmed
// present by grepping that exact file before use, not assumed.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13, reused from Stage 0 as a "still alive" heartbeat

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

static void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2/PA3 -> alternate function mode (MODER = 10), AF7 (USART2)
    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));

    USART2->BRR = 0x0683u; // 9600 baud @ 16MHz HSI, OVER8=0 -- see header comment
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE; // 8N1 (M=0 reset default), TX only
}

static void usart2_send_byte(uint8_t byte) {
    while (!(USART2->SR & USART_SR_TXE)) {
        // wait for the transmit data register to be free
    }
    USART2->DR = byte;
}

static void usart2_send_string(const char *s) {
    while (*s) {
        usart2_send_byte((uint8_t)*s++);
    }
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u)); // general purpose output

    usart2_init();

    while (1) {
        usart2_send_string("EdgeNeuro Stage 3a: UART alive\r\n");
        GPIOC->ODR ^= (1u << LED_PIN);
        delay(1600000u); // rough ~1s-ish toggle at 16MHz HSI, not calibrated
    }
}

// Stage 3b: ADC1 channel 0 (PA0), continuous conversion via software
// trigger, polled (no Timer, no DMA, no interrupts yet) -- isolates the
// ADC itself from Timer-driven periodic triggering, same incremental
// discipline as Stage 3a isolating UART from everything else. Reports the
// raw 12-bit reading over the already-verified USART2 link (see
// uart_hello_main.c) so it can be read on a real screen instead of only
// inferred from an LED pattern.
//
// PA0 is already wired to MyoWare 2.0's ENV pin (see README's MyoWare
// wiring table) -- if MyoWare isn't attached yet, PA0 floats and the
// printed value will be noisy/unstable, which is expected and still
// proves the ADC mechanism itself works; touching the wire to 3.3V or GND
// is a simple manual sanity check without needing the sensor attached.
//
// Every register value below is verified against the official STM32F401
// reference manual (RM0368 Rev 5), not guessed:
//   - RCC_APB2ENR bit 8 = ADC1EN (RM0368 6.3.12)
//   - RCC_AHB1ENR bit 0 = GPIOAEN (RM0368 6.3.9, same as Stage 3a)
//   - GPIOx_MODER: 11 = analog mode for a pin (RM0368 Table 24 /
//     8.4.1) -- PA0 is bits[1:0]
//   - ADC_CR2: bit 0 = ADON, bit 1 = CONT, bit 30 = SWSTART (RM0368
//     11.12.3). EXTEN left at reset (00, external trigger disabled) --
//     software-triggered only, no Timer involved in this stage.
//   - ADC_SQR3: SQ1[4:0] = bits[4:0] = channel number for the 1st (and
//     here, only) conversion in the regular sequence (RM0368 11.12.11).
//     Channel 0 = reset value 0, written explicitly anyway for clarity.
//   - ADC_SQR1: L[3:0] = bits[23:20] = 0000 -> 1 conversion in the
//     sequence (RM0368 11.12.9). Also already the reset value.
//   - ADC_SR bit 1 = EOC, end of conversion; cleared by reading ADC_DR
//     (RM0368 11.12.1 / 11.3.4 "managing a sequence... without DMA").
//   - ADC_DR bits[15:0] = DATA, right-aligned 12-bit result by default
//     (ALIGN=0 reset value) (RM0368 11.12.14).
// All register field macros (RCC_APB2ENR_ADC1EN, ADC_CR2_ADON, etc.) were
// confirmed present by grepping the same CMSIS device header already used
// in Stage 3a, not assumed.

#include "stm32f4xx.h"

static void delay(volatile uint32_t count) {
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

    USART2->BRR = 0x0683u; // 9600 baud @ 16MHz HSI, OVER8=0 -- see uart_hello_main.c
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
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

// Minimal unsigned-to-decimal-ASCII, no libc dependency (matches this
// project's "no newlib formatted I/O in the hot path" stance).
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

static void adc1_init_pa0(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // already on from usart2_init, harmless to repeat
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    GPIOA->MODER &= ~(3u << (0u * 2u));
    GPIOA->MODER |= (3u << (0u * 2u)); // PA0 -> analog mode (11)

    ADC1->SQR1 &= ~ADC_SQR1_L;   // L[3:0] = 0000 -> 1 conversion in the sequence
    ADC1->SQR3 &= ~ADC_SQR3_SQ1; // SQ1[4:0] = 0 -> channel 0 (PA0 = ADC1_IN0) first/only

    ADC1->CR2 |= ADC_CR2_ADON; // power up, wait for stabilization before starting
    delay(10000u);             // rough margin, not calibrated -- same style as Stage 0's delay
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_SWSTART; // free-running conversion, software-triggered
}

static uint32_t adc1_read_latest(void) {
    while (!(ADC1->SR & ADC_SR_EOC)) {
        // wait for the current conversion to finish
    }
    return ADC1->DR & 0xFFFu; // reading DR also clears EOC
}

int main(void) {
    usart2_init();
    adc1_init_pa0();

    while (1) {
        usart2_send_string("PA0 ADC1_IN0 raw = ");
        usart2_send_uint(adc1_read_latest());
        usart2_send_string("\r\n");
        delay(1600000u); // rough ~1s-ish pacing, not calibrated -- same style as Stage 0
    }
}

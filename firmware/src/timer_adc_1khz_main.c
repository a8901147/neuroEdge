// Stage 3c/3d: TIM2 drives ADC1 channel 0 (PA0) at a hardware-timed 1kHz
// rate via TRGO -> EXTSEL, answering unknown #3 for real (Stage 3a/3b only
// proved UART and ADC work in isolation; this is the first time Timer and
// ADC are combined, which is what "1kHz sampling" actually means). No
// software polling loop decides when to sample -- TIM2's update event
// triggers each ADC1 conversion directly in hardware, so the sample rate
// only depends on the Timer's PSC/ARR configuration, not on how fast the
// main loop happens to run.
//
// Every register value below is verified against the official STM32F401
// reference manual (RM0368 Rev 5), not guessed:
//   - RCC_APB1ENR bit 0 = TIM2EN (RM0368 6.3.11, same table as USART2EN)
//   - TIM2 is a 32-bit counter (RM0368 13.2) -- generous headroom, not
//     that it matters here since ARR fits in 16 bits anyway.
//   - TIM2CLK: "if the APB prescaler division factor is 1, TIMxCLK = HCLK.
//     Otherwise, TIMxCLK = 2xPCLKx" (RM0368 6.2, Figure 12 note). This
//     project's clock is unconfigured (default reset state, APB1
//     prescaler = 1, same 16MHz HSI as every other stage so far), so
//     TIM2CLK = HCLK = 16MHz exactly -- no x2 multiplier applies.
//   - Update event rate = TIM2CLK / ((PSC+1) x (ARR+1)) (RM0368 13.3.1).
//     For exactly 1000 Hz at 16MHz: (PSC+1) x (ARR+1) = 16000. Chose
//     PSC=15 (PSC+1=16), ARR=999 (ARR+1=1000) -> 16 x 1000 = 16000,
//     giving an EXACT integer division (no fractional rounding error,
//     unlike Stage 3a's UART baud rate) -- accuracy is limited only by
//     the HSI oscillator's own (uncalibrated, ~1%) tolerance, not by this
//     divider math.
//   - TIM_CR2 bits[6:4] = MMS, 010 = "Update" event selected as TRGO
//     output (RM0368 13.4.2) -- this is what lets TIM2's update event
//     reach the ADC as a hardware trigger, no CPU/interrupt involved.
//   - TIM_CR1 bit 0 = CEN, counter enable (RM0368 13.4.1).
//   - ADC_CR2 bits[29:28] = EXTEN, 01 = trigger detection on the rising
//     edge (RM0368 11.12.3).
//   - ADC_CR2 bits[27:24] = EXTSEL, 0110 = "Timer 2 TRGO event" as the
//     external trigger source for the regular group (RM0368 11.12.3,
//     same register page as EXTEN -- this exact encoding table is what's
//     being relied on here, not general STM32 knowledge).
//   - ADC_SR bit 1 = EOC; reading ADC_DR clears it (RM0368 11.12.1,
//     reused from Stage 3b).
// All register field macros (RCC_APB1ENR_TIM2EN, TIM_CR2_MMS_1,
// ADC_CR2_EXTEN_0, etc.) confirmed present in the CMSIS device header
// already used by every prior stage, not assumed.

#include "stm32f4xx.h"

#define LED_PIN 13u // PC13, toggled once per report -- see main()'s comment on why

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

static void gpioa_pa0_analog(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // already on from usart2_init, harmless to repeat
    GPIOA->MODER &= ~(3u << (0u * 2u));
    GPIOA->MODER |= (3u << (0u * 2u)); // PA0 -> analog mode (11)
}

static void tim2_init_1khz_trgo(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 15u;  // (15+1) x (999+1) = 16 x 1000 = 16000 -> exactly 1kHz @ 16MHz TIM2CLK
    TIM2->ARR = 999u;
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1; // MMS=010, Update event -> TRGO
    TIM2->CR1 |= TIM_CR1_CEN; // start counting; TRGO pulses begin firing at 1kHz from here
}

static void adc1_init_timer_triggered(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->SQR1 &= ~ADC_SQR1_L;   // 1 conversion in the regular sequence
    ADC1->SQR3 &= ~ADC_SQR3_SQ1; // channel 0 (PA0 = ADC1_IN0)

    ADC1->CR2 |= ADC_CR2_ADON; // power up, wait for stabilization
    delay(10000u);             // rough margin, not calibrated -- same style as Stage 3b

    // EXTEN=01 (rising edge), EXTSEL=0110 (Timer 2 TRGO event). No SWSTART,
    // no CONT -- each TRGO pulse triggers exactly one conversion.
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL)) |
                ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u)); // general purpose output, same as Stage 0

    usart2_init();
    gpioa_pa0_analog();
    adc1_init_timer_triggered();
    tim2_init_1khz_trgo();

    uint32_t sample_count = 0;
    uint32_t latest = 0;

    while (1) {
        if (ADC1->SR & ADC_SR_EOC) {
            latest = ADC1->DR & 0xFFFu; // reading DR also clears EOC
            ++sample_count;
        }

        // Report every 1000 samples rather than every sample: at 9600
        // baud, printing a full line per 1ms sample would flood the UART
        // (9600 baud is ~960 bytes/s, nowhere near enough for 1000
        // lines/s) -- see this file's header comment. Reporting once per
        // 1000 samples has a second purpose: if TIM2 is really firing at
        // 1kHz, this print (and the LED toggle next to it) should happen
        // roughly once per second -- a rough but real empirical check on
        // the achieved rate, not just a compiled-and-hoped-for one.
        if (sample_count > 0 && sample_count % 1000u == 0u) {
            usart2_send_string("samples=");
            usart2_send_uint(sample_count);
            usart2_send_string(" latest_adc=");
            usart2_send_uint(latest);
            usart2_send_string("\r\n");
            GPIOC->ODR ^= (1u << LED_PIN);
        }
    }
}

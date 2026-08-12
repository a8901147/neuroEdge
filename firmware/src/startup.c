// Minimal Cortex-M4 startup: vector table + reset handler. No CMSIS
// startup file, no HAL — matches the project's "understand every byte in
// the hot path" ethos, and keeps Stage 0 free of anything we can't
// explain. Only NMI/HardFault/SysTick etc. and Reset are wired up; nothing
// yet uses interrupts, so unimplemented vectors just spin in Default_Handler
// if hardware ever calls one (a sign something's misconfigured, not real work).

#include <stdint.h>

extern uint32_t _sidata; // start of .data's LOAD address in Flash
extern uint32_t _sdata;  // start of .data in RAM
extern uint32_t _edata;  // end of .data in RAM
extern uint32_t _sbss;   // start of .bss
extern uint32_t _ebss;   // end of .bss
extern uint32_t _estack; // top of stack (end of RAM)

int main(void);

void Reset_Handler(void);
void Default_Handler(void);

void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector"), used))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))&_estack, // initial stack pointer
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0, // reserved
    SVC_Handler,
    DebugMon_Handler,
    0, // reserved
    PendSV_Handler,
    SysTick_Handler,
    // Peripheral IRQ vectors intentionally omitted for Stage 0 (blink only
    // uses polling, no interrupts armed) -- Stage 3 (ADC/Timer/DMA) will
    // extend this table.
};

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    // Point the CPU at OUR vector table (g_pfnVectors, linked at Flash's
    // start per the linker script -- 0x08004000, after WeAct's HID
    // bootloader). Without this, SCB->VTOR is left wherever the bootloader
    // set it (its own table at 0x08000000), so any interrupt/exception
    // that fires after handoff -- SysTick, ADC, DMA, all still unused in
    // Stage 0/1 but load-bearing from Stage 3 onward -- would look up the
    // bootloader's handlers instead of ours.
    #define SCB_VTOR (*(volatile uint32_t *)0xE000ED08u)
    SCB_VTOR = (uint32_t)&g_pfnVectors;

    main();
    while (1) {
        // main() must never return on a system with no OS to return to.
    }
}

void Default_Handler(void) {
    while (1) {
        // An unimplemented exception/interrupt fired -- stop here so it's
        // obvious under a debugger which vector was missing, rather than
        // silently jumping to garbage.
    }
}

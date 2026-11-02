// Board-level helpers for phase3_control_loop: busy-wait delay and the PC13 LED.
// Included only by phase3_control_loop_main.cpp (one translation unit).
#pragma once

#include <cstdint>

#include "stm32f4xx.h"

#define LED_PIN 13u

static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

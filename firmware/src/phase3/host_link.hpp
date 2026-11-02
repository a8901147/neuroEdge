// USART2 link to the PC (115200 8N1): the queued tick/diag output and the commands the PC sends.
// Included only by phase3_control_loop_main.cpp (one translation unit).
//
// Commands (one byte at a time, parsed without blocking):
//   T<on>[,<release>]\n  EMG grip/release thresholds (emg_threshold_link.hpp)
//   R                    walk every servo back to the start pose
//   C<20 values>,<sum>\n MeArm calibration (mearm_calibration_link.hpp)
//   O<bits>\n            optional sensors, accepted only during the first ~300 ms after boot
#pragma once

#include <cstddef>
#include <cstdint>

#include "edgeneuro/control/emg_threshold_link.hpp"
#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/mearm_calibration_link.hpp"
#include "edgeneuro/control/tx_ring.hpp"
#include "stm32f4xx.h"

// Which IMUs must answer at boot. Both by default: a board nobody is driving halts on a wiring fault instead of
// silently running without a sensor. read_optional_sensors_config() may relax this.
static bool g_require_shoulder_imu = true;
static bool g_require_elbow_imu = true;

static void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));

    // 115200 baud from the 16 MHz HSI (no file configures RCC->CFGR/PLLCFGR). RM0368 19.3.4, OVER8=0:
    // USARTDIV = 16e6 / (16 * 115200) = 8.6875 -> mantissa 8, fraction round(0.6875 * 16) = 11 -> BRR 0x8B
    // (115,108 baud, 0.08 % error).
    USART2->BRR = 0x008Bu;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

// Set in main() right after the grip state machine is constructed; main() never returns, so the pointer stays valid.
static edgeneuro::GripStateMachine<float> *g_grip_for_threshold_update = nullptr;
static edgeneuro::emg_threshold_link::Parser g_threshold_parser;
static volatile bool g_home_requested = false;   // set by an 'R' byte, consumed by the servo block
// A complete, checksummed calibration raises g_cal_received; the servo block validates it before use.
#if EDGENEURO_DRIVE_SERVOS
static edgeneuro::mearm::calibration_link::Parser g_cal_parser;
static volatile bool g_cal_received = false;
#endif
static uint32_t g_cal_applied = 0u;
static uint32_t g_cal_rejected = 0u;      // complete messages whose calibration Path B refused

// Reads at most one received byte and hands it to every command parser. Sends nothing: it is also called from inside
// the blocking send's TXE wait, where a reply would interleave with the line being sent.
static void poll_threshold_update(void) {
    if (USART2->SR & USART_SR_RXNE) {
        const uint8_t b = (uint8_t)USART2->DR;
#if EDGENEURO_DRIVE_SERVOS
        if (g_cal_parser.feed(b)) {
            g_cal_received = true;
        }
#endif
        if (b == (uint8_t)'R') {
            g_home_requested = true;
        }
        if (g_threshold_parser.feed(b) && g_grip_for_threshold_update != nullptr) {
            g_grip_for_threshold_update->set_thresholds((float)g_threshold_parser.on(),
                                                        (float)g_threshold_parser.release());
        }
    }
}

// Once the main loop runs, output is queued and usart2_pump() moves one byte per loop pass when TXE allows. Waiting
// on every byte of a ~346-byte line (~30 ms at 115200) had cut the 1 kHz loop to 291 passes/s (measured 2026-10-03).
// Boot messages and the fatal blink path still send blocking, since nothing pumps the queue there. A line that does
// not fit is skipped whole (uart_line_fits), never sent half.
static edgeneuro::TxRing<2048> g_tx;
static bool g_tx_queued = false;          // false until the main loop starts
// Upper bounds per queued line (measured 2026-10-03: tick line ~346 bytes, diag line ~480). A tick or EDGE line is
// queued only if a diag line still fits after it.
static constexpr std::size_t kTickLineMax = 512u;
static constexpr std::size_t kDiagLineMax = 640u;
static constexpr std::size_t kEdgeLineMax = 32u;
static uint32_t g_rx_overruns = 0u;       // USART SR.ORE seen: a received byte was lost before it was read

static void usart2_pump(void) {
    const uint32_t sr = USART2->SR;
    if (sr & USART_SR_ORE) {
        ++g_rx_overruns;                  // cleared by the DR read in poll_threshold_update() (SR then DR)
    }
    poll_threshold_update();
    uint8_t b = 0u;
    if ((sr & USART_SR_TXE) && g_tx.pop(b)) {
        USART2->DR = b;
    }
}

static bool uart_line_fits(std::size_t max_len) {
    return !g_tx_queued || g_tx.begin_line(max_len);
}

static void usart2_send_byte_blocking(uint8_t byte) {
    // USART2 has no RX FIFO: keep reading received bytes while waiting, or a command arriving during a long boot
    // message would be overwritten before anything reads it.
    while (!(USART2->SR & USART_SR_TXE)) {
        poll_threshold_update();
    }
    USART2->DR = byte;
}

static void usart2_send_byte(uint8_t byte) {
    if (g_tx_queued) {
        g_tx.push(byte);                  // a refused byte is counted by TxRing; lines are gated so it should not happen
    } else {
        usart2_send_byte_blocking(byte);
    }
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

static void usart2_send_int(int32_t value) {
    if (value < 0) {
        usart2_send_byte('-');
        usart2_send_uint((uint32_t)(-value));
    } else {
        usart2_send_uint((uint32_t)value);
    }
}

// Waits up to 300 ms (counted in 1 kHz ADC conversions, already running) for "O<bits>\n": bit0 = shoulder IMU
// optional, bit1 = elbow IMU optional. Must run before the IMU wake-up, which halts on a required sensor that does not
// answer. Without a command both stay required. Reading ADC1->DR here is safe: nothing else reads it before the loop.
static void read_optional_sensors_config(void) {
    uint32_t ticks = 0;
    bool have_o = false;
    uint32_t bits = 0;
    while (ticks < 300u) {
        if (USART2->SR & USART_SR_RXNE) {
            const uint8_t b = (uint8_t)USART2->DR;
            if (b == (uint8_t)'O') {
                have_o = true;
                bits = 0;
            } else if (have_o) {
                if (b >= (uint8_t)'0' && b <= (uint8_t)'9') {
                    bits = bits * 10u + (uint32_t)(b - (uint8_t)'0');
                } else if (b == (uint8_t)'\n') {
                    g_require_shoulder_imu = (bits & 0x1u) == 0u;
                    g_require_elbow_imu = (bits & 0x2u) == 0u;
                    usart2_send_string("OPTIONAL_SENSORS_OK bits=");
                    usart2_send_uint(bits);
                    usart2_send_string("\r\n");
                    return;
                }
                // any other byte mid-line is ignored, not an error.
            }
        }
        if (ADC1->SR & ADC_SR_EOC) {
            (void)ADC1->DR;
            ++ticks;
        }
    }
}

// Fixed-point decimal printing (no printf float support in this freestanding build), 6 decimals to match the CSV
// replay's output so tools/mujoco_bridge/run_demo.py's LINE_RE parses both. The scaled value fits in uint32_t for
// |value| < 4294; everything printed here (angles, g, gyro at most 4.4 rad/s) stays below 5.
static void usart2_send_float(float value, uint32_t decimals = 6u) {
    if (value < 0.0f) {
        usart2_send_byte('-');
        value = -value;
    }
    uint32_t scale = 1u;
    for (uint32_t i = 0; i < decimals; ++i) {
        scale *= 10u;
    }
    const uint32_t scaled = (uint32_t)(value * (float)scale + 0.5f);
    usart2_send_uint(scaled / scale);
    usart2_send_byte('.');
    const uint32_t frac = scaled % scale;
    uint32_t pad = scale / 10u;
    while (pad > 0u && frac < pad) {
        usart2_send_byte('0');
        pad /= 10u;
    }
    if (frac > 0u) {
        usart2_send_uint(frac);
    }
}

// On-target latency of the EdgeNeuro pipeline (2026-10-08): the same two configurations as the host Google Benchmark
// (include/edgeneuro/bench/latency_configs.hpp, shared), timed per tick with the Cortex-M4's DWT cycle counter.
//
// Clock: the default HSI, 16 MHz, untouched (no PLL; nothing in this file writes RCC->CFGR/PLLCFGR), i.e. what every
// firmware in this repo runs at. Compiled with this repo's firmware flags (-Os, -mfpu=fpv4-sp-d16 -mfloat-abi=hard).
// Input: synthetic, recycled from a 64-sample buffer here (EDGENEURO_BENCH_SAMPLES; 256 on the host -- see the
// header for why the per-tick work is the same).
//
// Output, once a second over USART2 (PA2, 115200 8N1) and in the volatile g_* results below (readable over SWD):
//   latency mode=1x6 n=... min=... mean_x100=... max=... classify_mean_x100=... overhead=... f_hz=16000000
// all in CPU cycles; overhead (two back-to-back CYCCNT reads) is reported, not subtracted. "classify" = the ticks that
// completed a window (every 50th) and ran MAV + LDA; the others only filter and buffer.
//
// Registers, each checked against the vendored CMSIS core_cm4.h (cmsis-core v5.9.0) before writing:
//   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk   (DEMCR bit 24, enables the DWT)
//   DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk                 (bit 25, set = no cycle counter: then this reports and stops)
//   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk               (bit 0, starts CYCCNT)
// The USART2 setup is copied from phase3_control_loop_main.cpp's usart2_init (verified there against RM0368).
#include "stm32f4xx.h"

#include <cstddef>
#include <cstdint>

#include "edgeneuro/bench/latency_configs.hpp"

namespace B = edgeneuro::bench;

namespace {

constexpr std::uint32_t kCoreHz = 16000000u;   // HSI, the reset default (RCC_CFGR read back 0 on this board, 2026-10-03)
constexpr std::uint32_t kWarmupTicks = 500u;
constexpr std::uint32_t kMeasuredTicks = 5000u;  // 100 windows of 50

void usart2_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));
    USART2->BRR = 0x008Bu;   // 115200 at 16 MHz (RM0368 19.3.4; see phase3_control_loop_main.cpp)
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void send_byte(std::uint8_t b) {
    while (!(USART2->SR & USART_SR_TXE)) {
    }
    USART2->DR = b;
}

void send(const char* s) {
    while (*s) send_byte(static_cast<std::uint8_t>(*s++));
}

void send_uint(std::uint32_t v) {
    char d[10];
    int n = 0;
    do {
        d[n++] = static_cast<char>('0' + v % 10u);
        v /= 10u;
    } while (v > 0u && n < 10);
    while (n > 0) send_byte(static_cast<std::uint8_t>(d[--n]));
}

bool cycle_counter_start() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) return false;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return true;
}

volatile std::size_t g_sink = 0;

struct Result {
    B::CycleStats all, classify;
};

template <class Engine>
Result measure(Engine& engine) {
    std::size_t out = 0;
    for (std::uint32_t i = 0; i < kWarmupTicks; ++i) engine.tick(out);
    Result r;
    for (std::uint32_t i = 0; i < kMeasuredTicks; ++i) {
        // Compiler barriers (like Google Benchmark's DoNotOptimize/ClobberMemory): the engine's state must be in
        // memory and no part of tick() may be moved outside the two counter reads.
        __asm__ volatile("" : : "r"(&engine), "r"(&out) : "memory");
        const std::uint32_t t0 = DWT->CYCCNT;
        __asm__ volatile("" : : : "memory");
        engine.tick(out);
        __asm__ volatile("" : : "r"(&engine), "r"(&out) : "memory");
        const std::uint32_t t1 = DWT->CYCCNT;
        r.all.add(t1 - t0);
        if (engine.has_result()) r.classify.add(t1 - t0);
    }
    g_sink = out;   // keep the result observable so nothing is optimized away
    return r;
}

std::uint32_t measure_overhead() {
    B::CycleStats s;
    for (int i = 0; i < 1000; ++i) {
        const std::uint32_t t0 = DWT->CYCCNT;
        const std::uint32_t t1 = DWT->CYCCNT;
        s.add(t1 - t0);
    }
    return s.min();
}

std::uint32_t x100(double v) { return static_cast<std::uint32_t>(v * 100.0 + 0.5); }

void report(const char* mode, const Result& r, std::uint32_t overhead) {
    send("latency mode=");
    send(mode);
    send(" n=");
    send_uint(r.all.count());
    send(" min=");
    send_uint(r.all.min());
    send(" mean_x100=");
    send_uint(x100(r.all.mean()));
    send(" max=");
    send_uint(r.all.max());
    send(" classify_mean_x100=");
    send_uint(x100(r.classify.mean()));
    send(" overhead=");
    send_uint(overhead);
    send(" f_hz=");
    send_uint(kCoreHz);
    send("\r\n");
}

void delay(std::uint32_t count) {
    while (count--) __asm__ volatile("nop");
}

}  // namespace

// Results for SWD (openocd read_memory): [count, min, mean_x100, max, classify_mean_x100] per mode, plus overhead.
volatile std::uint32_t g_latency_wearable[5];
volatile std::uint32_t g_latency_hd[5];
volatile std::uint32_t g_latency_overhead = 0xFFFFFFFFu;
volatile std::uint32_t g_latency_done = 0u;   // 1 = measured; 0xDEAD = this core has no cycle counter

int main(void) {
    usart2_init();
    if (!cycle_counter_start()) {
        g_latency_done = 0xDEADu;
        while (true) {
            send("latency error: DWT has no cycle counter (DWT_CTRL.NOCYCCNT set)\r\n");
            delay(1600000u);
        }
    }
    const std::uint32_t overhead = measure_overhead();

    auto wearable = B::make_wearable_engine();
    const Result rw = measure(wearable);
    auto hd = B::make_hd_engine();
    const Result rh = measure(hd);

    const Result* results[2] = {&rw, &rh};
    volatile std::uint32_t* outs[2] = {g_latency_wearable, g_latency_hd};
    for (int m = 0; m < 2; ++m) {
        outs[m][0] = results[m]->all.count();
        outs[m][1] = results[m]->all.min();
        outs[m][2] = x100(results[m]->all.mean());
        outs[m][3] = results[m]->all.max();
        outs[m][4] = x100(results[m]->classify.mean());
    }
    g_latency_overhead = overhead;
    g_latency_done = 1u;

    while (true) {
        report("1x6", rw, overhead);
        report("32x0", rh, overhead);
        delay(1600000u);
    }
}

// Stage 5b: the real Phase 3 control loop shape -- EMG (Stage 5a,
// verified working on real hardware) and IMU orientation sharing one
// 1kHz main loop, IMU read as a non-blocking state machine instead of
// the blocking mpu6050_read_regs() every prior IMU stage used.
//
// Why non-blocking is mandatory here, not just nicer: EMG's TIM2+ADC1
// trigger fires every 1ms (hardware-verified, PRD.md Stage 3c/3d). A
// full 14-byte I2C read at 100kHz takes ~1.5-2ms -- longer than one tick
// -- so doing it as one blocking call would stall EMG sampling for more
// than a full tick period every time. Instead, mpu6050_read_step() below
// advances the read by checking at most one hardware flag per call and
// returning immediately either way; the main loop calls it once per EMG
// tick, and a full reading completes opportunistically across however
// many ticks the bus actually needs (in practice 2-4 ticks) without ever
// blocking any single tick.
//
// dt for ComplementaryFilter::update() is real elapsed ticks since the
// last completed reading (kDtPerTick=0.001f, from the same TIM2-verified
// exact period Stage 5a's kDt used) -- not a guess, and not the same
// value every call, since reads don't complete on a fixed schedule.
//
// ImuReader's target address: originally validated against the
// LCD1602/PCF8574 backpack's address (0x27) while both original
// MPU6050/GY-521 units were dead (PRD.md Stage 4b) -- the LCD ACKs
// address+W, accepts any byte as if it were a register address (PCF8574
// doesn't have registers, it just latches GPIO state), and ACKs address+R,
// returning whatever's on its input pins. Not real sensor data, but real
// I2C bus timing that exercises the exact same START/ADDR/TXE/BTF/RXNE/STOP
// sequence, including the multi-byte-read BTF tail timing that was the
// actual bug in Stage 4a's original blocking implementation, which is what
// gave confidence this state machine's protocol logic was correct before a
// working MPU6050 was available to test against directly. Stage 5c
// replaced it with one real Adafruit MPU-6050 at 0x68 (AD0 low). Stage 6
// (this version) adds a second real unit at 0x69 (AD0 tied to 3.3V) on the
// same bus -- ImuReader now takes its target address as a constructor
// argument instead of a single hard-coded global, since one instance per
// address is needed.

#include <cmath>
#include <cstdint>

#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/slew_rate_limiter.hpp"
#include "edgeneuro/fusion/complementary_filter.hpp"
#include "stm32f4xx.h"

#define LED_PIN 13u

// --- EMG side (unchanged constants from Stage 5a) ---
// 2026-09-09: the boot-time interactive relax/clench calibration (compile-
// time kCalibrationEnabled, a blocking usart2_recv_byte() gate before the
// main loop could even start) is GONE -- see git history for the full
// arc (2026-08-18 kFallbackThreshold=1220 -> 08-23 2037 -> 09-08 2800,
// then a same-day detour through kCalibrationEnabled=true with a settle
// skip + smoothing window to fight ADC jitter, all superseded by this).
// Root problem with the boot-gate design: it blocked EVERYTHING (EMG AND
// IMU streaming) on a human answering two prompts within one specific boot
// window, and that window kept getting missed for reasons that had nothing
// to do with EMG at all (a flaky UART link, a reflash landing before
// anything was listening) -- which is exactly the kind of coupling
// shoulder calibration never had, since raw_shoulder_ax/ay/az stream
// unconditionally from boot and Python decides when to average a window,
// completely independent of whether the main loop has started.
// Same fix shape now applies to EMG: kFallbackThreshold below is what
// GripStateMachine boots with (same value grip_state_machine uses at
// startup regardless of anything Python does), and
// tools/mujoco_bridge/run_demo_live.py computes a real relax/clench
// threshold on the HOST from the emg_min/emg_max this loop already streams
// every tick unconditionally, then pushes it live via the "T<uint>\n"
// command parsed non-blockingly in the main loop below (see
// g_pending_threshold_line/apply_pending_threshold_line()) --
// GripStateMachine::set_threshold() applies it without any reset or
// restart. No boot gate, nothing to hang on, no reflash required to
// recalibrate.
// Recalibrated 2026-08-23: the old 2037 was set against a baseline of ~450
// (Stage 5a, 2026-08-18). The MyoWare's onboard gain trim pot has since
// drifted/been bumped -- confirmed via adc_hello_main.c raw ADC readings
// that baseline is now ~2000-2150 relaxed vs. ~3600+ contracted (same real
// signal, just a different gain setting), so the old threshold sat inside
// the relaxed range and risked false-triggering "gripping" at rest. 2800
// sits with margin above the new relaxed baseline and below sustained
// contraction -- but is now only ever the BOOT default; a real session is
// expected to immediately push a fresh live value from run_demo_live.py.
static constexpr float kFallbackThreshold = 2800.0f;
static constexpr float kOnDuration = 0.15f;
static constexpr float kOffDuration = 0.15f;

// --- Which sensors THIS session's bench setup actually has wired up
// (2026-09-07) ---
// Originally both IMUs were unconditionally required: a failed wake write
// (PWR_MGMT_1 write never ACKed) called blink_code() -- a real infinite
// while(1), never returns -- which halted the WHOLE loop before EMG ever
// ran, since the wake writes happen before the main loop starts. That was
// fine when every bring-up session had both IMUs connected, but is wrong
// the moment someone wants to bench-test just the EMG chain (exactly what
// happened first: real hardware sat there printing nothing at all, for
// ten seconds, with a perfectly good MyoWare connected, because neither
// IMU was wired up and the firmware never got past its own boot gate to
// find out). Set to false for a sensor genuinely absent this session; a
// wake failure for a sensor still marked true still halts (blink_code) --
// that's a real wiring fault worth catching loudly, not something to
// silently downgrade to a warning. Edit + reflash to change which sensors
// a given bench session needs (this is bare-metal firmware -- there's no
// argv to make it a real runtime flag the way
// tools/mujoco_bridge/run_demo_live.py's --optional-sensors is; unlike the
// EMG threshold above, there's no live-streamed raw signal a host could use
// to make this decision after the fact -- whether a wake write ACKs has to
// be decided before the main loop exists at all).
// Set false<->true here to match whatever's ACTUALLY wired up before each
// reflash. Back to true/true (2026-09-08): both IMUs are back on the
// breadboard alongside MyoWare, so a real wiring fault should halt loudly
// again instead of being silently tolerated -- was false/false for one
// session (2026-09-07) while bench-testing MyoWare alone.
static constexpr bool kRequireShoulderImu = true;
static constexpr bool kRequireElbowImu = true;
static constexpr float kSlewRate = 5.0f;
static constexpr float kDtPerTick = 0.001f; // TIM2-verified exact 1kHz

// --- IMU side ---
// Stage 6: two real MPU6050 units share one I2C1 bus, distinguished by
// address -- the upper-arm unit keeps AD0 low (default, 0x68, same unit
// validated in Stage 5c); the forearm unit's AD0 is tied to 3.3V, giving
// 0x69 (MPU6050 datasheet: AD0 is bit 0 of the 7-bit address). Both units
// are the same sensor, so kImuRegAddr (ACCEL_XOUT_H) is shared.
static constexpr uint8_t kShoulderImuAddr = 0x68u; // upper-arm MPU6050, AD0 low
static constexpr uint8_t kElbowImuAddr = 0x69u;    // forearm MPU6050, AD0 tied to 3.3V
static constexpr uint8_t kImuRegAddr = 0x3Bu;    // ACCEL_XOUT_H -- meaningless against the LCD, kept for protocol shape
static constexpr uint32_t kImuReadLen = 14u;
static constexpr uint32_t kImuMaxTicksPerRead = 50u; // abort+retry a read stuck > 50ms

// I2C1 Fast Mode (400kHz, register math below) was tried once a real
// MPU6050 arrived (PRD.md Stage 5c) and measured a real ~3.4x throughput
// win (592/s -> 2028/s completions) -- but a second test on the same
// wiring immediately after a physical disturbance (shaking the board to
// look at IMU jitter) got stuck BUSY at 400kHz on wiring that worked fine
// at 100kHz moments earlier, reproducibly. Conclusion: the chip and this
// register math both support 400kHz, but this breadboard/jumper-wire
// setup's parasitic capacitance likely pushes real SCL/SDA rise times
// past Fast mode's tighter 300ns budget (vs. Sm mode's 1000ns) even when
// TRISE is computed correctly for it -- not a math bug, a real signal-
// integrity limit of this prototyping hardware. 100kHz stays the default
// for reliability; kept the Fast-mode constants (unused now) since the
// speed win is real if this ever moves to a soldered/shorter-trace board.
// RM0368 18.6.8/18.6.9: Fast-mode period=3*CCR*Tpclk1 (DUTY=0, not Sm
// mode's 2*CCR*Tpclk1), max rise time 300ns. At 16MHz Tpclk1=62.5ns:
// CCR=16,000,000/400,000/3=13.33, rounded UP to 14 (13 would give
// ~410kHz, over the Fast-mode max) -> ~381kHz. TRISE=(300/62.5)+1=5.8,
// rounded up to 6. F/S=1 (bit15) selects Fast mode; DUTY=0 (bit14).
static constexpr uint32_t kI2cCcrFastMode400k = 0x800Eu;  // F/S=1, DUTY=0, CCR=14 -- unused, see above
static constexpr uint32_t kI2cTriseFastMode400k = 6u;     // unused, see above
static constexpr uint32_t kI2cCcr100k = 0x50u;
static constexpr uint32_t kI2cTrise100k = 0x11u;

// Diagnostic-only globals, readable via `openocd ... mdw` -- see PRD.md
// Stage 5b debugging notes.
volatile int g_imu_state_at_timeout = -1;
volatile uint32_t g_sr1_at_timeout = 0xFFFFFFFFu;
volatile uint32_t g_sr2_at_timeout = 0xFFFFFFFFu;
volatile uint32_t g_timeout_count = 0;
// Stage 6: one wake-write result and one completion counter per IMU,
// replacing the single-IMU g_wake_result/imu_completions -- needed to
// confirm both addresses actually ACK independently (see PRD.md Stage 6
// verification notes) rather than assuming a single passing check covers
// both physical units.
volatile int g_wake_result_shoulder = -1;
volatile int g_wake_result_elbow = -1;

static void delay(uint32_t count) {
    while (count--) {
        __asm__ volatile("nop");
    }
}

// --- USART2 (unchanged from every prior stage) ---

static void usart2_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~((3u << (2u * 2u)) | (3u << (3u * 2u)));
    GPIOA->MODER |= (2u << (2u * 2u)) | (2u << (3u * 2u));
    GPIOA->AFR[0] &= ~((0xFu << (4u * 2u)) | (0xFu << (4u * 3u)));
    GPIOA->AFR[0] |= (7u << (4u * 2u)) | (7u << (4u * 3u));

    // Stage 6: 115200 baud (was 9600) -- the new dual-IMU output line is
    // ~95 bytes; at 9600 baud (~960 B/s at 8N1) that caps out around
    // 10 lines/sec, too choppy for smooth arm tracking. PCLK1 is HSI
    // 16MHz, unconfigured (no file in this project touches RCC->CFGR/
    // RCC->PLLCFGR -- confirmed by grep, not assumed), same clock the
    // original 9600-baud BRR was derived against. RM0368 19.3.4 Eq. 1
    // (Baud = fCK / (16 * USARTDIV), OVER8=0): USARTDIV = 16,000,000 /
    // (16*115200) = 8.6875 -> Mantissa=8 (0x8), Fraction=round(0.6875*16)
    // =11 (0xB) -> BRR=(0x8<<4)|0xB=0x8B. Actual baud ~=115,108 (0.08%
    // error, well inside UART tolerance).
    USART2->BRR = 0x008Bu;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

// Live EMG threshold update: parses a "T<digits>\n" line arriving
// asynchronously over UART and applies it via GripStateMachine::set_threshold()
// -- see kFallbackThreshold's own comment (near the top of this file) for
// the full design this replaces (a boot-time blocking calibration gate).
// g_grip_for_threshold_update is set once, right after `grip` is
// constructed in main() -- a raw pointer to a stack object is safe here
// because main() never returns (this whole program is the one function
// call), same reasoning as this file's other several volatile globals used
// purely for cross-cutting diagnostics/state.
static edgeneuro::GripStateMachine<float> *g_grip_for_threshold_update = nullptr;
static bool g_threshold_line_active = false;
static uint32_t g_threshold_line_value = 0;

// Deliberately does NOT send any acknowledgment string: found the hard way
// (2026-09-09) that this function is called from INSIDE usart2_send_byte's
// own TXE busy-wait below (see that function) -- if applying a completed
// "T<digits>\n" line tried to usart2_send_string() an ack right there, that
// would recursively call usart2_send_byte while the OUTER call is still
// mid-transmission, garbling whichever line was already in flight. Silent
// apply is fine: the caller doesn't need a reply to know it landed (see
// send_emg_threshold() in run_demo_live.py).
static void poll_threshold_update(void) {
    if (USART2->SR & USART_SR_RXNE) {
        const uint8_t b = (uint8_t)USART2->DR;
        if (b == (uint8_t)'T') {
            g_threshold_line_active = true;
            g_threshold_line_value = 0;
        } else if (g_threshold_line_active) {
            if (b >= (uint8_t)'0' && b <= (uint8_t)'9') {
                g_threshold_line_value = g_threshold_line_value * 10u + (uint32_t)(b - (uint8_t)'0');
            } else if (b == (uint8_t)'\n') {
                if (g_grip_for_threshold_update != nullptr) {
                    g_grip_for_threshold_update->set_threshold((float)g_threshold_line_value);
                }
                g_threshold_line_active = false;
            }
            // any other byte mid-line (e.g. a stray '\r') is ignored, not
            // an error -- keeps this parser tiny.
        }
    }
}

static void usart2_send_byte(uint8_t byte) {
    // 2026-09-09: polls for an incoming threshold-update byte while
    // otherwise just spinning here -- without this, a short RX burst
    // (e.g. "T2691\n", ~52us at 115200 baud) arriving entirely during one
    // of this loop's own ~95-byte line transmissions (~8ms at 115200 baud)
    // would be silently lost: USART2->DR has no RX FIFO, so multiple bytes
    // arriving before anything reads DR just overwrite each other. Found
    // on real hardware: a threshold update sent while the tick=/[DIAG]
    // stream was running never took effect, 100% of the time, until this
    // fix -- polling only at the top of the main loop (this function's
    // only caller besides poll_threshold_update itself) left RX starved
    // for however long each print call blocked.
    while (!(USART2->SR & USART_SR_TXE)) {
        poll_threshold_update();
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

static void usart2_send_int(int32_t value) {
    if (value < 0) {
        usart2_send_byte('-');
        usart2_send_uint((uint32_t)(-value));
    } else {
        usart2_send_uint((uint32_t)value);
    }
}

// Stage 6: real decimal-point printing (e.g. "-0.123456"), replacing the
// prior x1000-scaled-integer helper this file used through Stage 5c --
// needed to match tools/mujoco_bridge/run_demo.py's LINE_RE regex, which is
// shared verbatim with the CSV-replay prototype (src/mujoco_bridge_demo.cpp)
// so both can drive the same Python parsing code. No <cstdio>/printf float
// support assumed available in this freestanding build -- built from the
// same hand-rolled digit-string style
// as usart2_send_uint/usart2_send_int above. `decimals` defaults to 6 to
// match the CSV prototype's own printed float precision; roll/pitch/elbow
// are radian-range (|value| < ~4), so `scaled` stays well inside uint32_t.
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

// --- EMG: ADC1 channel 0 (PA0) via TIM2 TRGO, verified Stage 3c/3d ---

static void gpioa_pa0_analog(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(3u << (0u * 2u));
    GPIOA->MODER |= (3u << (0u * 2u));
}

static void tim2_init_1khz_trgo(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15u;
    TIM2->ARR = 999u;
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void adc1_init_timer_triggered(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3 &= ~ADC_SQR3_SQ1;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(10000u);
    ADC1->CR2 = (ADC1->CR2 & ~(ADC_CR2_EXTEN | ADC_CR2_EXTSEL)) |
                ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;
}

// --- IMU: I2C1 (PB6=SCL, PB7=SDA), verified Stage 4a driver, blocking
// helpers kept only for init/SWRST -- the read itself is non-blocking,
// see ImuReader below. ---

// Blocking register write. Originally a genuinely one-shot boot call (see
// main()'s wake-up comment); also called from i2c1_bus_recovery() below as
// of 2026-08-23, since a device that lost power mid-run (not just a stuck
// bus) comes back up freshly power-on-reset -- PWR_MGMT_1 defaults to
// SLEEP=1, so its accel/gyro registers stop updating even though I2C reads
// keep completing normally (real ACKs, real protocol, stale data forever).
// Bus recovery alone can't tell "stuck bus" apart from "device power-
// cycled", so it re-sends this wake write defensively every time; harmless
// on an already-awake device (idempotent). Still bounded/rare enough to
// stay blocking rather than folding into ImuReader's non-blocking design.
// Returns 0 on success.
static int mpu6050_write_reg_blocking(uint8_t addr7, uint8_t reg, uint8_t value) {
    uint32_t guard = 100000u;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if (--guard == 0) return 1;
    }

    I2C1->DR = (uint8_t)(addr7 << 1);
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (I2C1->SR1 & I2C_SR1_AF) {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 2;
        }
        if (--guard == 0) return 2;
    }
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_TXE)) {
        if (--guard == 0) return 3;
    }
    I2C1->DR = reg;
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--guard == 0) return 3;
    }

    I2C1->DR = value;
    guard = 100000u;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if (--guard == 0) return 4;
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

// Shared by i2c1_init() (fresh boot) and i2c1_bus_recovery() below (after a
// wedged bus is cleared) -- factored out since both need the exact same
// SWRST + reconfigure sequence, previously duplicated inline in both places
// plus a third time in ImuReader::step()'s timeout branch.
static void i2c1_swrst_recover(void) {
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16u;
    I2C1->CCR = kI2cCcr100k;
    I2C1->TRISE = kI2cTrise100k;
    I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOB->MODER |= (2u << (6u * 2u)) | (2u << (7u * 2u));
    GPIOB->OTYPER |= (1u << 6u) | (1u << 7u);
    GPIOB->PUPDR &= ~((3u << (6u * 2u)) | (3u << (7u * 2u)));
    GPIOB->PUPDR |= (1u << (6u * 2u)) | (1u << (7u * 2u));
    GPIOB->AFR[0] &= ~((0xFu << (4u * 6u)) | (0xFu << (4u * 7u)));
    GPIOB->AFR[0] |= (4u << (4u * 6u)) | (4u << (4u * 7u));

    i2c1_swrst_recover();
}

// PB6=SCL, PB7=SDA -- MODER-only helper: OTYPER (open-drain) and PUPDR
// (pull-up), both set once by i2c1_init() above, apply regardless of MODER,
// so handing a pin between the I2C1 peripheral (AF, mode 10) and plain
// bit-banged GPIO (general-purpose output, mode 01) only needs this.
static void gpiob_pin_set_mode(uint32_t pin, uint32_t mode) {
    GPIOB->MODER &= ~(3u << (pin * 2u));
    GPIOB->MODER |= (mode << (pin * 2u));
}

volatile uint32_t g_bus_recovery_attempts = 0;
volatile uint32_t g_bus_recovery_freed = 0;

// I2C-bus specification (NXP UM10204) sec 3.1.16, "Bus clear": if a slave
// is stuck holding SDA low, a master-side SWRST alone can't fix it -- SWRST
// only resets *our* I2C1 peripheral, not the external chip physically
// holding the line low. The documented fix is for the master to take SCL
// over as a manual GPIO and clock it up to 9 times (enough to walk a slave
// stuck anywhere in a byte+ACK through releasing SDA), then issue a STOP.
// Confirmed necessary 2026-08-23: unplugging only VIN/GND (not SDA/SCL)
// from one MPU6050 left it powered-off with its I2C pins still wired to the
// live, pulled-up bus -- if that happens mid-transaction, the now-unpowered
// output stage can freeze holding SDA low, wedging the shared bus for both
// readers and re-timing-out forever instead of self-healing via plain
// SWRST (see the plain-SWRST comment removed from this function's call site
// in ImuReader::step()).
static bool i2c1_bus_recovery(void) {
    g_bus_recovery_attempts = g_bus_recovery_attempts + 1;

    I2C1->CR1 &= ~I2C_CR1_PE; // release peripheral control of the pins
    gpiob_pin_set_mode(6u, 1u); // SCL -> general-purpose output
    gpiob_pin_set_mode(7u, 1u); // SDA -> general-purpose output

    GPIOB->ODR |= (1u << 7u); // let SDA float high (open-drain + pull-up)
    GPIOB->ODR |= (1u << 6u); // SCL high
    delay(2000u);

    bool freed = (GPIOB->IDR & (1u << 7u)) != 0u;
    for (int i = 0; i < 9 && !freed; ++i) {
        GPIOB->ODR &= ~(1u << 6u); // SCL low
        delay(2000u);
        GPIOB->ODR |= (1u << 6u); // SCL high -- clock edge for a stuck slave
        delay(2000u);
        freed = (GPIOB->IDR & (1u << 7u)) != 0u;
    }
    if (freed) {
        g_bus_recovery_freed = g_bus_recovery_freed + 1;
    }

    // STOP condition: SDA low->high while SCL high, so any slave watching
    // sees a clean bus-idle handoff rather than an ambiguous mid-clock stop.
    GPIOB->ODR &= ~(1u << 7u);
    delay(2000u);
    GPIOB->ODR |= (1u << 6u);
    delay(2000u);
    GPIOB->ODR |= (1u << 7u);
    delay(2000u);

    gpiob_pin_set_mode(6u, 2u); // SCL back to AF (I2C1)
    gpiob_pin_set_mode(7u, 2u); // SDA back to AF (I2C1)
    i2c1_swrst_recover();

    // Re-wake both sensors defensively (see this function's docstring):
    // covers the case where the wedge was actually a device power-cycling,
    // which resets it to SLEEP=1 and would otherwise leave it reporting
    // "online" (real ACKs, completions counting up) with permanently frozen
    // accel/gyro data -- the exact symptom of reads succeeding but the
    // MuJoCo view never moving after a recovery.
    mpu6050_write_reg_blocking(kShoulderImuAddr, 0x6Bu, 0x01u);
    mpu6050_write_reg_blocking(kElbowImuAddr, 0x6Bu, 0x01u);

    return freed;
}

// Non-blocking multi-byte I2C1 read state machine. step() checks at most
// one hardware flag and returns immediately regardless of outcome -- call
// it once per EMG tick. Mirrors i2c_mpu6050_hello_main.c's verified
// mpu6050_read_regs() protocol sequence exactly, just spread across many
// calls instead of busy-waiting inside one.
enum class ImuReadState : uint8_t {
    Idle,
    WaitStart1,
    WaitAddr1,
    WaitRegTxe,
    WaitRegBtf,
    WaitStart2,
    WaitAddr2,
    ReadingMiddle,
    WaitBtfPenultimate,
    WaitBtfLast,
};

class ImuReader {
public:
    // Stage 6: target address is a constructor argument (was a single
    // hard-coded kImuTargetAddr global) so one instance can be created per
    // physical MPU6050 sharing this I2C1 bus.
    explicit ImuReader(uint8_t addr7) : addr7_(addr7) {}

    // start_tick: the caller's EMG-tick counter at the moment this read
    // began -- used only for the timeout below. step() itself is called
    // far more often than once per EMG tick (see main()'s loop, which
    // polls I2C as fast as the CPU can spin between ADC conversions, not
    // gated to the 1kHz tick), so counting *calls* to step() would make
    // the timeout fire in a fraction of a millisecond instead of the
    // intended ~50ms -- comparing against real elapsed EMG ticks instead
    // keeps the timeout meaning what it says regardless of poll rate.
    void begin(uint32_t start_tick) {
        state_ = ImuReadState::WaitStart1;
        start_tick_ = start_tick;
        byte_index_ = 0;
        I2C1->CR1 |= I2C_CR1_START;
    }

    bool is_idle() const { return state_ == ImuReadState::Idle; }
    const uint8_t *buf() const { return buf_; }
    int state_as_int() const { return (int)state_; } // diagnostics only
    // Per-instance, unlike g_timeout_count/etc below which are shared across
    // both readers -- needed to tell "this specific device's address stopped
    // ACKing" (nack_count_) apart from "the whole I2C1 peripheral wedged"
    // (timeout_count_, since a wedged bus stalls whichever reader happens to
    // be active when it happens, not necessarily the one whose device is
    // actually the problem).
    uint32_t nack_count() const { return nack_count_; }
    uint32_t timeout_count() const { return timeout_count_; }
    void reset_counts() { nack_count_ = 0; timeout_count_ = 0; }

    // Design principle (2026-08-25): don't infer "is this device actually
    // awake" from a symptom (e.g. a suspicious-looking reading) -- track it
    // as an explicit fact instead. Any time THIS reader's own address fails
    // to ACK, that's unambiguous: this specific device stopped responding,
    // for whatever reason (power blip, physical disconnect, ...), and on a
    // real MPU6050 that can include a silent power-on-reset back to
    // SLEEP=1. So the next time it completes a read, the caller must
    // unconditionally re-send the wake-up write before trusting the data --
    // not just when the data happens to look wrong. consume_needs_rewake()
    // is check-and-clear so this fires exactly once per failure, not on
    // every completion forever after.
    bool consume_needs_rewake() {
        bool v = needs_rewake_;
        needs_rewake_ = false;
        return v;
    }

    // current_tick: the caller's current EMG-tick counter, for the same
    // timeout reason as begin() above.
    // Returns true exactly on the call a full kImuReadLen-byte reading
    // completes (buf() then holds it, valid until the next begin()).
    bool step(uint32_t current_tick) {
        if (state_ == ImuReadState::Idle) {
            return false;
        }
        if (current_tick - start_tick_ > kImuMaxTicksPerRead) {
            g_imu_state_at_timeout = (int)state_;
            g_sr1_at_timeout = I2C1->SR1;
            g_sr2_at_timeout = I2C1->SR2;
            g_timeout_count = g_timeout_count + 1;
            ++timeout_count_;
            needs_rewake_ = true;
            // A plain SWRST isn't enough to recover a slave that's
            // physically holding SDA low (e.g. it lost power mid-
            // transaction while still wired to the bus, see
            // i2c1_bus_recovery()'s docstring) -- SWRST only resets our own
            // I2C1 peripheral, not the external chip. i2c1_bus_recovery()
            // does the documented I2C bus-clear (manual SCL clock-out) and
            // then performs the same SWRST+reconfigure this used to do
            // inline, so a genuinely wedged bus self-recovers on the next
            // begin() instead of requiring a manual reflash.
            I2C1->CR1 |= I2C_CR1_STOP;
            i2c1_bus_recovery();
            state_ = ImuReadState::Idle;
            return false;
        }

        switch (state_) {
        case ImuReadState::Idle:
            return false;

        case ImuReadState::WaitStart1:
            if (I2C1->SR1 & I2C_SR1_SB) {
                I2C1->DR = (uint8_t)(addr7_ << 1); // address + W
                state_ = ImuReadState::WaitAddr1;
            }
            return false;

        case ImuReadState::WaitAddr1:
            if (I2C1->SR1 & I2C_SR1_ADDR) {
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                state_ = ImuReadState::WaitRegTxe;
            } else if (I2C1->SR1 & I2C_SR1_AF) {
                I2C1->SR1 &= ~I2C_SR1_AF;
                I2C1->CR1 |= I2C_CR1_STOP;
                state_ = ImuReadState::Idle;
                ++nack_count_;
                needs_rewake_ = true;
            }
            return false;

        case ImuReadState::WaitRegTxe:
            if (I2C1->SR1 & I2C_SR1_TXE) {
                I2C1->DR = kImuRegAddr;
                state_ = ImuReadState::WaitRegBtf;
            }
            return false;

        case ImuReadState::WaitRegBtf:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 |= I2C_CR1_ACK;
                I2C1->CR1 |= I2C_CR1_START; // repeated START
                state_ = ImuReadState::WaitStart2;
            }
            return false;

        case ImuReadState::WaitStart2:
            if (I2C1->SR1 & I2C_SR1_SB) {
                I2C1->DR = (uint8_t)((addr7_ << 1) | 1u); // address + R
                state_ = ImuReadState::WaitAddr2;
            }
            return false;

        case ImuReadState::WaitAddr2:
            if (I2C1->SR1 & I2C_SR1_ADDR) {
                (void)I2C1->SR1;
                (void)I2C1->SR2;
                byte_index_ = 0;
                state_ = ImuReadState::ReadingMiddle;
            } else if (I2C1->SR1 & I2C_SR1_AF) {
                I2C1->SR1 &= ~I2C_SR1_AF;
                I2C1->CR1 |= I2C_CR1_STOP;
                state_ = ImuReadState::Idle;
                ++nack_count_;
                needs_rewake_ = true;
            }
            return false;

        case ImuReadState::ReadingMiddle:
            if (I2C1->SR1 & I2C_SR1_RXNE) {
                buf_[byte_index_] = (uint8_t)I2C1->DR;
                ++byte_index_;
                if (byte_index_ == kImuReadLen - 2u) {
                    state_ = ImuReadState::WaitBtfPenultimate;
                }
            }
            return false;

        case ImuReadState::WaitBtfPenultimate:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 &= ~I2C_CR1_ACK;
                buf_[kImuReadLen - 2u] = (uint8_t)I2C1->DR;
                state_ = ImuReadState::WaitBtfLast;
            }
            return false;

        case ImuReadState::WaitBtfLast:
            if (I2C1->SR1 & I2C_SR1_BTF) {
                I2C1->CR1 |= I2C_CR1_STOP;
                buf_[kImuReadLen - 1u] = (uint8_t)I2C1->DR;
                state_ = ImuReadState::Idle;
                return true; // full reading complete
            }
            return false;
        }
        return false;
    }

private:
    uint8_t addr7_;
    ImuReadState state_{ImuReadState::Idle};
    uint32_t start_tick_{0};
    uint32_t byte_index_{0};
    uint8_t buf_[kImuReadLen]{};
    uint32_t nack_count_{0};
    uint32_t timeout_count_{0};
    bool needs_rewake_{false};
};

static int16_t be16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void blink_code(int code) {
    while (1) {
        for (int i = 0; i < code; ++i) {
            GPIOC->ODR &= ~(1u << LED_PIN);
            delay(150000u);
            GPIOC->ODR |= (1u << LED_PIN);
            delay(150000u);
        }
        delay(1200000u);
    }
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (LED_PIN * 2u));
    GPIOC->MODER |= (1u << (LED_PIN * 2u));

    usart2_init();
    gpioa_pa0_analog();
    adc1_init_timer_triggered();
    tim2_init_1khz_trgo();
    i2c1_init();
    usart2_send_string("Stage 5b: combined EMG+IMU 1kHz loop, non-blocking I2C\r\n");

    // Wake both sensors: PWR_MGMT_1 (0x6B) defaults to SLEEP=1 on power-up,
    // where accel/gyro registers don't update -- without this, ImuReader
    // reads complete "successfully" (real ACKs, real protocol) but return
    // all-zero/stale data forever. CLKSEL=001 (PLL w/ X-gyro reference)
    // per InvenSense's recommendation over the reset-default internal
    // oscillator (RM-MPU-6000A-00), same as every other stage that reads
    // real MPU6050 data. Stage 6: two independent wake writes, one per
    // address -- each device has its own PWR_MGMT_1 register, so waking
    // one has no effect on the other.
    g_wake_result_shoulder = mpu6050_write_reg_blocking(kShoulderImuAddr, 0x6Bu, 0x01u);
    if (g_wake_result_shoulder != 0) {
        usart2_send_string("shoulder MPU6050 (0x68) wake write FAILED, code=");
        usart2_send_int(g_wake_result_shoulder);
        usart2_send_string("\r\n");
        if constexpr (kRequireShoulderImu) {
            blink_code(9);
        } else {
            usart2_send_string("shoulder IMU marked optional (kRequireShoulderImu=false) "
                                "-- continuing without it\r\n");
        }
    }
    g_wake_result_elbow = mpu6050_write_reg_blocking(kElbowImuAddr, 0x6Bu, 0x01u);
    if (g_wake_result_elbow != 0) {
        usart2_send_string("elbow MPU6050 (0x69) wake write FAILED, code=");
        usart2_send_int(g_wake_result_elbow);
        usart2_send_string("\r\n");
        if constexpr (kRequireElbowImu) {
            blink_code(10);
        } else {
            usart2_send_string("elbow IMU marked optional (kRequireElbowImu=false) "
                                "-- continuing without it\r\n");
        }
    }

    // 2026-09-09: no boot-time calibration block here anymore -- see
    // kFallbackThreshold's own comment above for the full story. Starts
    // with the fallback; tools/mujoco_bridge/run_demo_live.py pushes a
    // real relax/clench-derived value live once it's connected and ready
    // (see poll_threshold_update() above, polled both from inside
    // usart2_send_byte's TXE wait and from the top of the main loop below).
    usart2_send_string("EMG threshold: fallback=");
    usart2_send_uint((uint32_t)kFallbackThreshold);
    usart2_send_string(" (awaiting live update over UART)\r\n");

    edgeneuro::GripStateMachine<float> grip(kFallbackThreshold, kOnDuration, kOffDuration);
    g_grip_for_threshold_update = &grip;
    edgeneuro::SlewRateLimiter<float> setpoint(kSlewRate);
    // Two independent filters, one per IMU -- ComplementaryFilter has no
    // static/global state (verified when this was first ported to the
    // Host-side src/mujoco_bridge_demo.cpp prototype), so two instances
    // don't cross-talk. No post-fusion IirFilter smoothing here (Stage 5b
    // had one, alpha=0.5, but PRD.md's own notes flag it as never actually
    // validated against real jitter) -- this matches the already-tested
    // Host-side prototype exactly, which also streams filter.roll()/
    // pitch() directly, so firmware and the Python-side math stay in
    // lockstep rather than diverging by an extra, unvalidated smoothing
    // stage.
    edgeneuro::ComplementaryFilter<float> shoulder_filter(0.98f, kDtPerTick);
    bool shoulder_filter_initialized = false;

    // Stage 6: two MPU6050s share I2C1, so their reads cannot run
    // concurrently -- only one ImuReader may have a transaction in flight
    // at a time. `active_is_shoulder` tracks which one.
    //
    // CORRECTNESS BUG (found 2026-08-23, fixed here): the first version of
    // this only flipped `active_is_shoulder` on a *successful* completion,
    // reasoning that a timeout should keep retrying the same device rather
    // than silently skipping it. That reasoning breaks badly the moment one
    // physical IMU actually goes away (unplugged, or the classic stuck-SDA
    // breadboard failure this project keeps hitting): the active reader
    // NACKs or times out, goes back to Idle, and `begin()`s again on the
    // very next pass -- on the SAME dead device, forever. The other,
    // perfectly healthy IMU never gets a turn again, so BOTH
    // ComplementaryFilters freeze (not just the broken one's), which is
    // exactly the confusing symptom that cost real debugging time: unplug
    // one sensor mid-run and *both* shoulder_pitch/roll and elbow go
    // static, with no crash and no obviously-missing data to point at the
    // real cause. Fixed below by switching turns whenever the active
    // reader goes idle for ANY reason -- success, NACK abort, or timeout
    // recovery -- not just success. A single failing IMU can still never
    // fully stall the other one this way; ImuReader's own retry/SWRST
    // recovery (unchanged) still gives a struggling-but-not-dead device a
    // fair chance to recover on its next turn.
    ImuReader shoulder_reader(kShoulderImuAddr);
    ImuReader elbow_reader(kElbowImuAddr);
    bool active_is_shoulder = true;

    uint32_t tick_count = 0;
    uint32_t shoulder_completions = 0;
    uint32_t elbow_completions = 0;
    uint32_t last_shoulder_completion_tick = 0;
    uint32_t emg_window_min = 0xFFFu;
    uint32_t emg_window_max = 0u;
    // Cumulative since boot -- counts times ImuReader::consume_needs_rewake()
    // fired true, meaning that reader's device had just failed to ACK at
    // some point and this is the first completion since. A device that
    // power-cycles mid-run (not just a wedged bus) comes back up freshly
    // power-on-reset with PWR_MGMT_1 back to its SLEEP=1 default -- I2C
    // reads keep completing cleanly (nacks=0, timeouts=0 once it's back),
    // but the accel/gyro registers were never written since reset, so
    // decoded readings sit at exactly (0,0,0) forever otherwise. Rather
    // than detect that symptom after the fact, needs_rewake_ makes "device
    // just recovered from a failure" the trigger, unconditionally, so any
    // future failure mode gets the same treatment without needing its own
    // special case.
    uint32_t shoulder_asleep_rewakes = 0;
    uint32_t elbow_asleep_rewakes = 0;

    float sp = 0.0f; // last EMG setpoint, for the periodic report below (updated only on EOC)

    // Raw, pre-remap ELBOW accelerometer axes -- printed alongside the
    // computed shoulder_pitch/shoulder_roll/elbow so a single UART line
    // carries both, letting a combined raw+simulated log correlate the two
    // directly (see tools/mujoco_bridge's combined_calibrate.py-style
    // scripts) instead of separate capture runs.
    float elbow_raw_ax = 0.0f;
    float elbow_raw_ay = 0.0f;
    float elbow_raw_az = 0.0f;
    // Temporary (2026-09-01): both sensors got remounted with a new,
    // consistent convention (pin-header edge facing the hand/distal
    // direction on both) -- the existing ax/ay/az remap below was derived
    // for the OLD mount and no longer applies to either reader. Re-add the
    // shoulder side's raw axes (removed once the last remount's remap was
    // confirmed) to re-derive it from scratch rather than re-guessing.
    float shoulder_raw_ax = 0.0f;
    float shoulder_raw_ay = 0.0f;
    float shoulder_raw_az = 0.0f;
    // Full 3-axis raw shoulder gyro, added 2026-09-05: gy/gz were already
    // read (as raw_gy/raw_gz below, used by the complementary filter) but
    // never stored/streamed, and raw_gx wasn't even read at all -- a live
    // debugging session needed the actual raw 6-axis stream (not just this
    // file's own post-decode pitch/roll numbers) to tell "the arm really
    // moved a lot" apart from "the algorithm mis-split a small motion",
    // and accel alone couldn't settle that.
    float shoulder_raw_gx = 0.0f;
    float shoulder_raw_gy = 0.0f;
    float shoulder_raw_gz = 0.0f;

    while (1) {
        // Also polled from inside usart2_send_byte's own TXE wait (see that
        // function's comment for why that's the fix that actually matters
        // -- this top-of-loop call covers the case where the loop is idle,
        // between ADC/I2C activity, not currently blocked in a print).
        poll_threshold_update();

        // --- IMU: advance whichever reader is currently active every pass
        // of this loop, not just once per EMG tick -- same throughput
        // rationale as Stage 5b (the CPU is otherwise idle between ADC
        // conversions). With two devices sharing one bus, expect roughly
        // half Stage 5c's single-IMU rate per device (~591-593/s measured
        // there -> ~290-295/s each here), since each transaction takes the
        // same bus time regardless of address -- re-measure via
        // shoulder_completions/elbow_completions below once wired, rather
        // than assuming.
        ImuReader &active_reader = active_is_shoulder ? shoulder_reader : elbow_reader;
        const bool was_idle_before_this_pass = active_reader.is_idle();
        if (was_idle_before_this_pass) {
            active_reader.begin(tick_count);
        }
        const bool completed = active_reader.step(tick_count);
        if (completed) {
            const uint8_t *b = active_reader.buf();
            const float raw_ax = (float)be16(&b[0]) / 16384.0f;
            const float raw_ay = (float)be16(&b[2]) / 16384.0f;
            const float raw_az = (float)be16(&b[4]) / 16384.0f;
            const float raw_gx = (float)be16(&b[8]) / 131.0f * (3.14159265f / 180.0f);
            const float raw_gy = (float)be16(&b[10]) / 131.0f * (3.14159265f / 180.0f);
            const float raw_gz = (float)be16(&b[12]) / 131.0f * (3.14159265f / 180.0f);

            if (active_is_shoulder) {
                shoulder_raw_ax = raw_ax;
                shoulder_raw_ay = raw_ay;
                shoulder_raw_az = raw_az;
                shoulder_raw_gx = raw_gx;
                shoulder_raw_gy = raw_gy;
                shoulder_raw_gz = raw_gz;
                ++shoulder_completions;
                if (active_reader.consume_needs_rewake()) {
                    ++shoulder_asleep_rewakes;
                    mpu6050_write_reg_blocking(kShoulderImuAddr, 0x6Bu, 0x01u);
                }
                // Axis remap for the shoulder mount (upper arm, near the
                // inner elbow, GY-521 chip face outward/away from skin,
                // -X toward the hand/distal direction) -- CORRECTED
                // 2026-09-03 after the 2026-09-01 remap below turned out
                // wrong for this mount: a live forward-raise test showed
                // the motion landing almost entirely on the filter's roll
                // output (0.1->1.9rad) while pitch barely moved (0.03->
                // -0.3rad, wrong sign too) -- see PRD.md/git history for
                // the full [CORR] trace this was diagnosed from.
                //
                // Geometric derivation (not just re-measured empirically --
                // this is the actual root cause the 2026-09-01 remap got
                // wrong): -X toward the hand means +X points toward the
                // shoulder (proximal) when the arm hangs at rest, so a
                // stationary accelerometer should read raw_ax ~= +1g at
                // rest (an axis pointing "up", opposing gravity, reads
                // +1g) -- confirmed directly, rest reading was raw_ax
                // ~= +0.72 to +0.83. As the arm flexes forward, X rotates
                // away from vertical toward horizontal, so raw_ax MUST
                // decrease -- confirmed, it dropped to ~-0.24 to -0.30 at
                // full forward raise. This is the axis that actually
                // carries the pitch signal; the 2026-09-01 remap instead
                // fed it into az (the shared reference/denominator used by
                // BOTH the pitch and roll formulas), which is why roll
                // picked up almost the entire motion instead of pitch --
                // feeding the most motion-sensitive axis into the role
                // meant to stay stable explains the bug mechanically, not
                // just by having re-measured and gotten different numbers.
                // raw_ay empirically stays close to flat through the same
                // motion (-0.58 to -0.65), consistent with it being the
                // axis largely uninvolved in pure forward/backward
                // rotation, so it takes over az's old (reference) role.
                //
                // ax NOT negated (unlike the previous remap): with
                // ax=raw_ax directly, pitch increased (-0.79->+0.24 by the
                // accel-angle formula) as the arm was raised forward,
                // already matching the "positive pitch = forward" data
                // convention (tools/mujoco_bridge/run_demo_live.py) with no
                // sign flip needed for this mount.
                //
                // NOT YET RE-VERIFIED: gx/gy gyro pairing below is left
                // UNCHANGED from the 2026-09-01 remap (gx=-raw_gy,
                // gy=raw_gz) -- that pairing was derived for the OLD
                // (wrong) ax/ay assignment and almost certainly needs its
                // own re-derivation now, but doing that from theory alone
                // needs the exact physical relationship between each accel
                // axis and its corresponding rotation-sensing gyro axis,
                // which isn't nailed down here -- left as-is rather than
                // guessed, per this project's standing rule against
                // speculative axis/register values. The complementary
                // filter's alpha=0.98 weighting means gyro dominates
                // short-term response (~0.5s time constant back to the
                // accel-implied angle), so a mismatched gyro pairing here
                // is a plausible remaining source of transient error even
                // though the corrected accel mapping above is the
                // dominant, verified fix. Re-derive by isolating gyro
                // channels the same way the accel channels were isolated
                // above (a clean, single-axis motion test), not by guessing.
                //
                // This mapping is shoulder-only: the elbow reader no
                // longer needs any axis remap or per-sensor Euler-angle
                // filter -- see elbow_bend_raw's computation below for why.
                const float ax = raw_ax;
                const float ay = raw_az;
                const float az = raw_ay;
                const float gx = -raw_gy;
                const float gy = raw_gz;
                if (!shoulder_filter_initialized) {
                    shoulder_filter.initialize(ax, ay, az); // skip the cold-start convergence transient
                    shoulder_filter_initialized = true;
                }
                const float dt = (float)(tick_count - last_shoulder_completion_tick) * kDtPerTick;
                last_shoulder_completion_tick = tick_count;
                shoulder_filter.update(gx, gy, ax, ay, az, dt); // real elapsed dt, not the fixed constructor value
            } else {
                elbow_raw_ax = raw_ax;
                elbow_raw_ay = raw_ay;
                elbow_raw_az = raw_az;
                ++elbow_completions;
                if (active_reader.consume_needs_rewake()) {
                    ++elbow_asleep_rewakes;
                    mpu6050_write_reg_blocking(kElbowImuAddr, 0x6Bu, 0x01u);
                }
            }
        }
        // Switch turns on a successful completion, OR when the reader that
        // was already busy at the top of this pass has now gone back to
        // idle without completing (NACK abort / timeout+SWRST recovery) --
        // see the bug writeup above shoulder_reader's declaration for why
        // this can't be success-only. Deliberately NOT triggered by the
        // was_idle_before_this_pass+begin() case in the same pass (a fresh
        // begin() is never idle again this same call), so a device that
        // starts a transaction this pass still gets to run it to
        // completion/failure before losing its turn.
        if (completed || (!was_idle_before_this_pass && active_reader.is_idle())) {
            active_is_shoulder = !active_is_shoulder;
        }

        if (ADC1->SR & ADC_SR_EOC) {
            ++tick_count;

            // --- EMG: unchanged from Stage 5a ---
            const uint32_t raw = ADC1->DR & 0xFFFu;
            if (raw < emg_window_min) emg_window_min = raw;
            if (raw > emg_window_max) emg_window_max = raw;

            const bool edge = grip.update((float)raw, kDtPerTick);
            sp = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDtPerTick);
            if (edge) {
                usart2_send_string(grip.is_gripping() ? "EDGE -> Gripping\r\n" : "EDGE -> Released\r\n");
            }

            // Stage 6: 100Hz (every 10 ticks), not 1kHz or the old 1Hz --
            // at 115200 baud a ~95-byte line supports up to ~120 lines/sec
            // (see usart2_init()'s BRR comment); 100Hz is comfortably
            // inside that budget and far more than MuJoCo's own
            // launch_passive loop needs, since it paces itself
            // independently to model.opt.timestep on the Python side.
            // Format matches tools/mujoco_bridge/run_demo.py's LINE_RE
            // exactly (shared with src/mujoco_bridge_demo.cpp's CSV-replay
            // prototype), so the same Python parsing code works unchanged
            // against either source.
            if (tick_count % 10u == 0u) {
                // 2026-09-01: elbow_bend is the angle between the shoulder
                // and elbow readers' raw (unmapped) gravity vectors, via
                // cos(angle) = (a.b) / (|a||b|) -- NOT
                // shoulder_filter.pitch() - elbow_filter.pitch() (removed;
                // see git history). That subtraction decomposes each
                // sensor's tilt into a per-axis Euler angle first, which
                // breaks down (the classic atan2 gimbal-lock singularity)
                // once either angle nears +-90deg -- exactly the range a
                // real ~140deg (2.44rad) elbow flexion has to cross.
                // Confirmed on hardware 2026-09-01: two different ax/ay
                // swap attempts at the elbow reader's per-axis mapping both
                // still showed the real motion landing mostly on "roll"
                // (swinging 2-3+ rad) while "pitch" barely moved (~0.4-1.2
                // rad), regardless of which raw channel fed which -- not a
                // mapping bug, a property of the decomposition itself at
                // this rotation size.
                //
                // The dot-product form has no such singularity (well-
                // behaved over its full 0-180deg range) and needs no axis
                // remap at all -- the angle between two vectors doesn't
                // care which coordinate frame each is expressed in, as long
                // as it's consistent per vector, so this uses each reader's
                // raw ax/ay/az directly. Always >= 0 (can't distinguish
                // flexion from the equivalent hyperextension), same
                // limitation the old ">0.0f" clamp already accepted.
                const float shoulder_mag = std::sqrt(shoulder_raw_ax * shoulder_raw_ax +
                                                      shoulder_raw_ay * shoulder_raw_ay +
                                                      shoulder_raw_az * shoulder_raw_az);
                const float elbow_mag = std::sqrt(elbow_raw_ax * elbow_raw_ax +
                                                   elbow_raw_ay * elbow_raw_ay +
                                                   elbow_raw_az * elbow_raw_az);
                float elbow_bend = 0.0f;
                // Guard against either reader's raw_* still sitting at its
                // zero-initialized default (no completion yet since boot) --
                // a zero-length vector makes the division below meaningless,
                // not just imprecise.
                if (shoulder_mag > 0.1f && elbow_mag > 0.1f) {
                    const float dot = shoulder_raw_ax * elbow_raw_ax + shoulder_raw_ay * elbow_raw_ay +
                                       shoulder_raw_az * elbow_raw_az;
                    // Clamp before acos: the dot-product identity can round
                    // to just past +-1 in float even for exactly-aligned
                    // vectors, and acos() of anything outside [-1,1] is
                    // NaN, not a clamped boundary value.
                    float cos_angle = dot / (shoulder_mag * elbow_mag);
                    if (cos_angle > 1.0f) cos_angle = 1.0f;
                    if (cos_angle < -1.0f) cos_angle = -1.0f;
                    elbow_bend = std::acos(cos_angle);
                }

                usart2_send_string("tick=");
                usart2_send_uint(tick_count);
                usart2_send_string(" grip=");
                usart2_send_float(sp);
                usart2_send_string(" gripping=");
                usart2_send_uint(grip.is_gripping() ? 1u : 0u);
                usart2_send_string(" shoulder_pitch=");
                usart2_send_float(shoulder_filter.pitch());
                usart2_send_string(" shoulder_roll=");
                usart2_send_float(shoulder_filter.roll());
                usart2_send_string(" elbow=");
                usart2_send_float(elbow_bend);
                // Temporary debug fields (2026-08-23): raw 12-bit ADC
                // min/max over the last ~10ms window, before any threshold
                // comparison. `grip`/`gripping` above are GripStateMachine's
                // OUTPUT (only moves once the raw signal clears
                // kFallbackThreshold=2037 for on_duration seconds), so they
                // can't distinguish "no real EMG signal reaching the ADC at
                // all" from "signal present but too weak to cross the
                // threshold" -- added to check which one this is. Remove
                // once confirmed one way or the other.
                usart2_send_string(" emg_min=");
                usart2_send_uint(emg_window_min);
                usart2_send_string(" emg_max=");
                usart2_send_uint(emg_window_max);
                usart2_send_string(" elbow_raw_ax=");
                usart2_send_float(elbow_raw_ax);
                usart2_send_string(" elbow_raw_ay=");
                usart2_send_float(elbow_raw_ay);
                usart2_send_string(" elbow_raw_az=");
                usart2_send_float(elbow_raw_az);
                // shoulder_raw_ax/ay/az: no longer just diagnostic (as of
                // 2026-09-01) -- these, together with elbow_raw_ax/ay/az
                // above, are the actual inputs elbow_bend's dot-product
                // angle is computed from, so kept printed as the ground
                // truth for that computation, not removed.
                usart2_send_string(" shoulder_raw_ax=");
                usart2_send_float(shoulder_raw_ax);
                usart2_send_string(" shoulder_raw_ay=");
                usart2_send_float(shoulder_raw_ay);
                usart2_send_string(" shoulder_raw_az=");
                usart2_send_float(shoulder_raw_az);
                // shoulder_raw_gx/gy/gz (added 2026-09-05): full raw
                // gyro, not just the gx/gy pairing the complementary
                // filter consumes internally -- see the field's own
                // declaration comment for why a live debugging session
                // needed this on the wire instead of only accel.
                usart2_send_string(" shoulder_raw_gx=");
                usart2_send_float(shoulder_raw_gx);
                usart2_send_string(" shoulder_raw_gy=");
                usart2_send_float(shoulder_raw_gy);
                usart2_send_string(" shoulder_raw_gz=");
                usart2_send_float(shoulder_raw_gz);
                usart2_send_string("\r\n");

                if (tick_count % 1000u == 0u) {
                    // Slower diagnostic-only line, same cadence Stage 5b
                    // used -- per-IMU completion counts for the
                    // verification checks in PRD.md's Stage 6 section
                    // (confirms the alternator is actually alternating,
                    // not stuck on one device). nack/timeout counts added
                    // 2026-08-23 to tell apart "this device's address
                    // stopped ACKing" (nacks, per-reader -- a real per-
                    // device signal) from "the whole I2C1 peripheral wedged
                    // BUSY" (timeouts -- can hit whichever reader happens to
                    // be active regardless of which device is actually at
                    // fault): unplugging IMU#2 was observed to also knock
                    // IMU#1 offline, which these numbers should distinguish
                    // between a real per-device electrical problem and a
                    // shared-bus glitch.
                    usart2_send_string("diag shoulder_completions=");
                    usart2_send_uint(shoulder_completions);
                    usart2_send_string(" elbow_completions=");
                    usart2_send_uint(elbow_completions);
                    usart2_send_string(" shoulder_nacks=");
                    usart2_send_uint(shoulder_reader.nack_count());
                    usart2_send_string(" shoulder_timeouts=");
                    usart2_send_uint(shoulder_reader.timeout_count());
                    usart2_send_string(" elbow_nacks=");
                    usart2_send_uint(elbow_reader.nack_count());
                    usart2_send_string(" elbow_timeouts=");
                    usart2_send_uint(elbow_reader.timeout_count());
                    usart2_send_string(" active_reader_state=");
                    usart2_send_uint((uint32_t)active_reader.state_as_int());
                    // Cumulative since boot, unlike the per-window counters
                    // above -- these fire rarely enough that "since boot"
                    // is more useful than resetting every window.
                    usart2_send_string(" bus_recovery_attempts=");
                    usart2_send_uint(g_bus_recovery_attempts);
                    usart2_send_string(" bus_recovery_freed=");
                    usart2_send_uint(g_bus_recovery_freed);
                    usart2_send_string(" shoulder_asleep_rewakes=");
                    usart2_send_uint(shoulder_asleep_rewakes);
                    usart2_send_string(" elbow_asleep_rewakes=");
                    usart2_send_uint(elbow_asleep_rewakes);
                    usart2_send_string("\r\n");
                    GPIOC->ODR ^= (1u << LED_PIN);
                    shoulder_completions = 0;
                    elbow_completions = 0;
                    shoulder_reader.reset_counts();
                    elbow_reader.reset_counts();
                }

                emg_window_min = 0xFFFu;
                emg_window_max = 0u;
            }
        }
    }
}

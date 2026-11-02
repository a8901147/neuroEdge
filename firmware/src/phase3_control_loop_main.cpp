// phase3_control_loop: the wearable controller. One 1 kHz polled loop on the STM32F401 (16 MHz HSI, no interrupts):
//   * EMG: the MyoWare envelope (ENV) on PA0, sampled by TIM2-triggered ADC1; EMA -> two-threshold grip state machine
//     -> slew-limited grip setpoint.
//   * IMUs: two MPU6050s on I2C1, read alternately by a non-blocking state machine (about 270 reads/s each).
//   * Servos (EDGENEURO_DRIVE_SERVOS builds): every 10th tick, the raw gravity vectors and the grip setpoint become
//     four MeArm pulse widths (mearm_drive.hpp), held when an IMU fails its health check.
//   * UART: a tick line for the PC at up to 100 Hz (about 34/s get through at 115200), a diag line once a second,
//     and the PC's commands (phase3/host_link.hpp).
// Hardware modules live in phase3/; the control logic is in include/edgeneuro/ and tested on the host.

#include <cmath>
#include <cstdint>

#ifndef EDGENEURO_DRIVE_SERVOS
#define EDGENEURO_DRIVE_SERVOS 0
#endif

// Which servos follow the sensors (bit0 base, bit1 shoulder, bit2 elbow, bit3 claw); the others hold rest. Default all.
#ifndef EDGENEURO_SERVO_MASK
#define EDGENEURO_SERVO_MASK 0xFu
#endif

#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/imu_health.hpp"
#include "edgeneuro/control/mpu6050_power_check.hpp"
#include "edgeneuro/control/mearm_drive.hpp"
#include "edgeneuro/control/mearm_joint_step.hpp"
#include "edgeneuro/control/mearm_input_filter.hpp"
#include "edgeneuro/control/mearm_calibration_link.hpp"
#include "edgeneuro/control/mearm_servo_maps.hpp"
#include "edgeneuro/control/slew_rate_limiter.hpp"
#include "edgeneuro/fusion/complementary_filter.hpp"
#include "phase3/board.hpp"
#include "phase3/emg_adc.hpp"
#include "phase3/host_link.hpp"
#include "phase3/mpu6050_i2c.hpp"
#if EDGENEURO_DRIVE_SERVOS
#include "phase3/servo_pwm.hpp"
#endif
#include "stm32f4xx.h"

// --- EMG grip ---
// Boot threshold in raw ADC counts, used until the PC sends a calibrated "T" line (run_demo_live.py computes it from
// a relaxed and a clenched recording). It sits above the relaxed level and below a sustained contraction at the
// MyoWare gain setting of 2026-08-23.
static constexpr float kFallbackThreshold = 2800.0f;
// The envelope must stay above (below) the threshold this long, without interruption, to grip (release).
static constexpr float kOnDuration = 0.15f;
static constexpr float kOffDuration = 0.15f;
// EMA on the ADC sample before the state machine (time constant ~10 ms at 1 kHz). Without it, a single noisy sample
// on the wrong side of the threshold restarts the uninterrupted 150 ms count, and the grip chattered near the
// threshold. Seeded with the first sample instead of 0, which is why it is not IirFilter (whose state starts at 0).
static constexpr float kEmgSmoothingAlpha = 0.1f;
// Grip setpoint slew rate over the 0..1 range: a full open/close takes 1 / 20 = 50 ms.
static constexpr float kSlewRate = 20.0f;
static constexpr float kDtPerTick = 0.001f; // one ADC conversion = one TIM2 period = 1 ms

// --- IMU power state, readable over SWD and printed in the diag line ---
// Last PWR_MGMT_1 read back from each sensor (0xFFFFFFFF = not read yet) and how often it read back reset or asleep.
volatile uint32_t g_pwr_mgmt_1_shoulder = 0xFFFFFFFFu;
volatile uint32_t g_pwr_mgmt_1_elbow = 0xFFFFFFFFu;
volatile uint32_t g_power_resets_shoulder = 0u;
volatile uint32_t g_power_resets_elbow = 0u;

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
#if EDGENEURO_DRIVE_SERVOS
    gpioa_tim3_ch1_ch2_af();
    gpiob_tim3_ch3_ch4_af();
    tim3_pwm_50hz_4ch_init();
#endif
    i2c1_init();

    // Clear the bus before the first transfer: at power-up an MPU6050 can still be holding SDA, and SWRST alone
    // (done by i2c1_init) does not release it.
    i2c1_bus_recovery();

    usart2_send_string("Stage 5b: combined EMG+IMU 1kHz loop, non-blocking I2C\r\n");

    read_optional_sensors_config();

    // Wake both sensors: PWR_MGMT_1 (0x6B) resets to SLEEP, where reads succeed but the data never changes.
    // CLKSEL = 1 (PLL with the X gyro as reference), as InvenSense recommends over the internal oscillator.
    g_wake_result_shoulder = mpu6050_write_reg_blocking(kShoulderImuAddr, 0x6Bu, 0x01u);
    if (g_wake_result_shoulder != 0) {
        usart2_send_string("shoulder MPU6050 (0x68) wake write FAILED, code=");
        usart2_send_int(g_wake_result_shoulder);
        usart2_send_string("\r\n");
        if (g_require_shoulder_imu) {
            blink_code(9);
        } else {
            usart2_send_string("shoulder IMU marked optional via O<bits> "
                                "-- continuing without it\r\n");
        }
    }
    g_wake_result_elbow = mpu6050_write_reg_blocking(kElbowImuAddr, 0x6Bu, 0x01u);
    if (g_wake_result_elbow != 0) {
        usart2_send_string("elbow MPU6050 (0x69) wake write FAILED, code=");
        usart2_send_int(g_wake_result_elbow);
        usart2_send_string("\r\n");
        if (g_require_elbow_imu) {
            blink_code(10);
        } else {
            usart2_send_string("elbow IMU marked optional via O<bits> "
                                "-- continuing without it\r\n");
        }
    }

    // DLPF on both, regardless of the wake result: a missing optional sensor just gets a failed write.
    mpu6050_write_reg_blocking(kShoulderImuAddr, kMpu6050ConfigReg, kDlpfCfg6);
    mpu6050_write_reg_blocking(kElbowImuAddr, kMpu6050ConfigReg, kDlpfCfg6);

    usart2_send_string("EMG threshold: fallback=");
    usart2_send_uint((uint32_t)kFallbackThreshold);
    usart2_send_string(" (awaiting live update over UART)\r\n");

    edgeneuro::GripStateMachine<float> grip(kFallbackThreshold, kOnDuration, kOffDuration);
    g_grip_for_threshold_update = &grip;
    edgeneuro::SlewRateLimiter<float> setpoint(kSlewRate);
    // Diagnostic only: its roll/pitch are streamed (check_hardware_ready.py --live-check watches them for frozen
    // data), but neither the servos nor the PC use them for control; both work from the raw gravity vectors. The gyro
    // axis pairing below was never re-derived after the shoulder sensor was remounted.
    edgeneuro::ComplementaryFilter<float> shoulder_filter(0.98f, kDtPerTick);
    bool shoulder_filter_initialized = false;
#if EDGENEURO_DRIVE_SERVOS
    // Each servo walks from its rest pulse to its target instead of jumping (servo_startup_ramp.hpp).
    struct {
        edgeneuro::ServoStartupRamp base, shoulder, elbow, claw;
    } ramps{edgeneuro::mearm::base_ramp(), edgeneuro::mearm::shoulder_ramp(), edgeneuro::mearm::elbow_ramp(),
            edgeneuro::mearm::claw_ramp()};
    edgeneuro::mearm::drive::Homing homing;   // started by 'R' over UART
    // The servo command is smoothed (1-euro filter on both IMUs, base hysteresis and slow follow while the arm is
    // raised); the ramps are only a safety limit. The UART keeps sending the raw readings.
    edgeneuro::mearm::ArmInputFilter arm_filter;
    edgeneuro::mearm::Hysteresis base_hold;
    edgeneuro::mearm::BaseRaiseFollow base_follow;
    // The compiled-in calibration, validated once at boot; nullptr (rejected by Path B) holds shoulder and elbow at
    // rest. A calibration received over UART replaces it.
    edgeneuro::mearm::pathb::Calibration arm_cal_storage;
    const edgeneuro::mearm::pathb::Calibration* arm_cal =
        edgeneuro::mearm::make_compiled_calibration(arm_cal_storage) ? &arm_cal_storage : nullptr;
    // The base's measured comfortable reach, or the default mapping if none was measured or it is unusable.
    edgeneuro::mearm::real::BaseReach base_reach{};
    if (arm_cal == nullptr || !edgeneuro::mearm::make_compiled_base_reach(*arm_cal, base_reach)) {
        base_reach = arm_cal ? edgeneuro::mearm::real::default_base_reach(*arm_cal) : edgeneuro::mearm::real::BaseReach{1.0f, -1.0f};
    }
    // A failed IMU stops the arm (implausible, saturated or frozen readings; imu_health.hpp). Sensors made optional
    // with O<bits> are not checked, so this is constructed after read_optional_sensors_config().
    edgeneuro::ImuHealth imu_health(g_require_shoulder_imu, g_require_elbow_imu);
#endif

    // Both IMUs share I2C1, so one reader runs at a time. The turn passes after every read that ends -- completed,
    // NACKed or timed out -- so a dead sensor cannot starve the healthy one.
    ImuReader shoulder_reader(kShoulderImuAddr);
    ImuReader elbow_reader(kElbowImuAddr);
    bool active_is_shoulder = true;
    // Once a second per sensor, the next read is its power registers instead of its data (mpu6050_power_check.hpp).
    edgeneuro::mpu6050::HealthSchedule shoulder_power_schedule(1000u);
    edgeneuro::mpu6050::HealthSchedule elbow_power_schedule(1000u);
    bool power_read_in_flight = false;

    uint32_t tick_count = 0;
    uint32_t shoulder_completions = 0;
    uint32_t elbow_completions = 0;
    uint32_t last_shoulder_completion_tick = 0;
    uint32_t emg_window_min = 0xFFFu;
    uint32_t emg_window_max = 0u;
    float emg_ema = 0.0f;
    bool emg_ema_initialized = false;
    // Re-wakes after a NACK or timeout (ImuReader::consume_needs_rewake), cumulative since boot.
    uint32_t shoulder_asleep_rewakes = 0;
    uint32_t elbow_asleep_rewakes = 0;

    float sp = 0.0f; // grip setpoint, updated every tick

    // Raw accelerometer vectors (g) in each sensor's own frame: the input of the elbow bend, the health check and
    // the servo block, and streamed for the PC, which decodes them the same way.
    float elbow_raw_ax = 0.0f;
    float elbow_raw_ay = 0.0f;
    float elbow_raw_az = 0.0f;
    float shoulder_raw_ax = 0.0f;
    float shoulder_raw_ay = 0.0f;
    float shoulder_raw_az = 0.0f;
    // Raw shoulder gyro (rad/s), streamed for analysis only.
    float shoulder_raw_gx = 0.0f;
    float shoulder_raw_gy = 0.0f;
    float shoulder_raw_gz = 0.0f;

    g_tx_queued = true;                   // from here on, UART output is queued and pumped (see usart2_pump)
    while (1) {
        usart2_pump();

        // --- IMU: advance the active reader on every pass, not only once per tick ---
        ImuReader &active_reader = active_is_shoulder ? shoulder_reader : elbow_reader;
        const bool was_idle_before_this_pass = active_reader.is_idle();
        if (was_idle_before_this_pass) {
            edgeneuro::mpu6050::HealthSchedule &sched = active_is_shoulder ? shoulder_power_schedule : elbow_power_schedule;
            power_read_in_flight = sched.due(tick_count);
            if (power_read_in_flight) {
                active_reader.begin(tick_count, edgeneuro::mpu6050::kHealthReadStartReg, edgeneuro::mpu6050::kHealthReadLen);
            } else {
                active_reader.begin(tick_count);
            }
        }
        const bool completed = active_reader.step(tick_count);
        if (completed && power_read_in_flight) {
            // A sensor that lost power between two reads comes back in SLEEP without any I2C error: wake it and set
            // its DLPF again.
            power_read_in_flight = false;
            const uint8_t pwr = active_reader.buf()[edgeneuro::mpu6050::kPwrMgmt1Index];
            const uint8_t addr = active_is_shoulder ? kShoulderImuAddr : kElbowImuAddr;
            (active_is_shoulder ? shoulder_power_schedule : elbow_power_schedule).mark_done(tick_count);
            if (active_is_shoulder) {
                g_pwr_mgmt_1_shoulder = pwr;
            } else {
                g_pwr_mgmt_1_elbow = pwr;
            }
            if (edgeneuro::mpu6050::classify_pwr_mgmt_1(pwr) != edgeneuro::mpu6050::PowerState::Awake) {
                if (active_is_shoulder) {
                    g_power_resets_shoulder = g_power_resets_shoulder + 1u;
                } else {
                    g_power_resets_elbow = g_power_resets_elbow + 1u;
                }
                mpu6050_write_reg_blocking(addr, 0x6Bu, edgeneuro::mpu6050::kPwrMgmt1Awake);
                mpu6050_write_reg_blocking(addr, kMpu6050ConfigReg, kDlpfCfg6);
            }
            active_is_shoulder = !active_is_shoulder;
        } else if (completed) {
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
                // Axis remap for the diagnostic filter only. On the upper arm, +X points toward the shoulder, so raw_ax
                // is ~+1 g with the arm hanging and falls as the arm is raised forward (measured: +0.72..+0.83 hanging,
                // -0.24..-0.30 raised); raw_ay barely changes in that motion and serves as the reference axis.
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
                shoulder_filter.update(gx, gy, ax, ay, az, dt); // real elapsed time: reads do not finish on a schedule
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
        // Pass the turn when a read ended this pass: completed, or the reader that was busy is idle again (NACK or
        // timeout). A read begun this pass is never idle again in the same pass, so it keeps its turn until it ends.
        if (completed || (!was_idle_before_this_pass && active_reader.is_idle())) {
            active_is_shoulder = !active_is_shoulder;
        }

        if (ADC1->SR & ADC_SR_EOC) {
            ++tick_count;

            // --- EMG ---
            const uint32_t raw = ADC1->DR & 0xFFFu;

            // The streamed min/max are of the RAW samples, before the EMA: they show the PC (calibration,
            // watch_emg_raw.py) the signal itself, glitches included.
            if (raw < emg_window_min) emg_window_min = raw;
            if (raw > emg_window_max) emg_window_max = raw;

            if (!emg_ema_initialized) {
                emg_ema = (float)raw;
                emg_ema_initialized = true;
            } else {
                emg_ema += kEmgSmoothingAlpha * ((float)raw - emg_ema);
            }

            const bool edge = grip.update(emg_ema, kDtPerTick);
            sp = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDtPerTick);
            if (edge && uart_line_fits(kEdgeLineMax + kDiagLineMax)) {
                usart2_send_string(grip.is_gripping() ? "EDGE -> Gripping\r\n" : "EDGE -> Released\r\n");
            }

            // Every 10th tick (100 Hz): the servo block and the tick line.
            if (tick_count % 10u == 0u) {
                // Elbow bend: the angle between the two raw gravity vectors, acos(a.b / |a||b|). Each vector is in its
                // own sensor's frame, so this equals the elbow angle only when the two frames coincide with the arm
                // straight and the elbow's axis is horizontal; a mounting misalignment adds a pose-dependent offset
                // (zero_elbow in the calibration removes it at one pose), and forearm rotation also changes it.
                // Accelerometer-only: always >= 0 and blind to bending about a vertical axis.
                const float shoulder_mag = std::sqrt(shoulder_raw_ax * shoulder_raw_ax +
                                                      shoulder_raw_ay * shoulder_raw_ay +
                                                      shoulder_raw_az * shoulder_raw_az);
                const float elbow_mag = std::sqrt(elbow_raw_ax * elbow_raw_ax +
                                                   elbow_raw_ay * elbow_raw_ay +
                                                   elbow_raw_az * elbow_raw_az);
                float elbow_bend = 0.0f;
                if (shoulder_mag > 0.1f && elbow_mag > 0.1f) {   // no reading yet from one of them
                    const float dot = shoulder_raw_ax * elbow_raw_ax + shoulder_raw_ay * elbow_raw_ay +
                                       shoulder_raw_az * elbow_raw_az;
                    float cos_angle = dot / (shoulder_mag * elbow_mag);
                    if (cos_angle > 1.0f) cos_angle = 1.0f;     // rounding can step just past +-1; acos would be NaN
                    if (cos_angle < -1.0f) cos_angle = -1.0f;
                    elbow_bend = std::acos(cos_angle);
                }

                // Servos (mearm_drive.hpp): Path B decode of the upper-arm vector with the current calibration ->
                // height/reach mapping inside the measured safe envelope; base from the azimuth; claw from the grip
                // setpoint. Then the start-up ramps, stepped every 10 ticks. Until both IMUs have reported, the health
                // check fails and every servo keeps the rest pulse set at init.
#if EDGENEURO_DRIVE_SERVOS
                imu_health.update({shoulder_raw_ax, shoulder_raw_ay, shoulder_raw_az},
                                  {elbow_raw_ax, elbow_raw_ay, elbow_raw_az});
                if (g_cal_received) {
                    g_cal_received = false;
                    if (edgeneuro::mearm::apply_calibration_message(g_cal_parser.values(), arm_cal_storage, base_reach)) {
                        arm_cal = &arm_cal_storage;
                        ++g_cal_applied;
                    } else {
                        ++g_cal_rejected;   // the arm keeps the calibration it had
                    }
                }
                if (g_home_requested) {
                    g_home_requested = false;
                    homing.request(ramps);
                }
                const auto filtered = arm_filter.update({shoulder_raw_ax, shoulder_raw_ay, shoulder_raw_az},
                                                        {elbow_raw_ax, elbow_raw_ay, elbow_raw_az}, kServoStepDt);
                auto sensed = edgeneuro::mearm::drive::command(arm_cal, filtered.upper, filtered.elbow_bend, sp,
                                                               imu_health.ok(), &base_reach);
                sensed.base = base_hold.apply(sensed.base, edgeneuro::mearm::kBaseHysteresisUs);
                sensed.base = base_follow.apply(sensed.base, arm_cal, filtered.upper, kServoStepDt);
                const auto cmd = homing.apply(edgeneuro::mearm::drive::only(sensed, EDGENEURO_SERVO_MASK), ramps);
                if (!cmd.hold) {    // a failed sensor: write nothing -- every servo keeps its pulse (stops moving)
                    TIM3->CCR1 = ramps.base.step(static_cast<float>(cmd.base), kServoStepDt);
                    // shoulder and elbow step together, so every intermediate pose stays inside the measured envelope
                    // (mearm_joint_step.hpp)
                    const auto se = edgeneuro::mearm::joint_step(ramps.shoulder, ramps.elbow, static_cast<float>(cmd.shoulder),
                                                                 static_cast<float>(cmd.elbow), kServoStepDt);
                    TIM3->CCR2 = se.shoulder;
                    TIM3->CCR3 = se.elbow;
                    TIM3->CCR4 = ramps.claw.step(static_cast<float>(cmd.claw), kServoStepDt);
                }
#endif

                // The tick line (format: run_demo.py's LINE_RE), sent only if it fits whole with room for a diag line.
                // At 115200 baud not every line fits; skipped lines are counted in the diag line.
                if (uart_line_fits(kTickLineMax + kDiagLineMax)) {
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
                // Raw 12-bit ADC min/max over the last ~10 ms: the input of the PC's EMG calibration.
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
                usart2_send_string(" shoulder_raw_ax=");
                usart2_send_float(shoulder_raw_ax);
                usart2_send_string(" shoulder_raw_ay=");
                usart2_send_float(shoulder_raw_ay);
                usart2_send_string(" shoulder_raw_az=");
                usart2_send_float(shoulder_raw_az);
                usart2_send_string(" shoulder_raw_gx=");
                usart2_send_float(shoulder_raw_gx);
                usart2_send_string(" shoulder_raw_gy=");
                usart2_send_float(shoulder_raw_gy);
                usart2_send_string(" shoulder_raw_gz=");
                usart2_send_float(shoulder_raw_gz);
                usart2_send_string("\r\n");
                }

                if (tick_count % 1000u == 0u) {
                    // Once a second: per-IMU read counts, NACKs (one device stopped answering) vs timeouts (the bus
                    // hung), wake results and required flags, bus recoveries, power state, UART and calibration counts.
                    if (uart_line_fits(kDiagLineMax)) {
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
                    // wake result: 0 = success, else mpu6050_write_reg_blocking's failure step;
                    // required = 1: a wake failure at boot halts (read_optional_sensors_config).
                    usart2_send_string(" shoulder_wake_result=");
                    usart2_send_int(g_wake_result_shoulder);
                    usart2_send_string(" elbow_wake_result=");
                    usart2_send_int(g_wake_result_elbow);
                    usart2_send_string(" shoulder_required=");
                    usart2_send_uint(g_require_shoulder_imu ? 1u : 0u);
                    usart2_send_string(" elbow_required=");
                    usart2_send_uint(g_require_elbow_imu ? 1u : 0u);
                    // Cumulative since boot (the counts above reset every second).
                    usart2_send_string(" bus_recovery_attempts=");
                    usart2_send_uint(g_bus_recovery_attempts);
                    usart2_send_string(" bus_recovery_freed=");
                    usart2_send_uint(g_bus_recovery_freed);
                    usart2_send_string(" shoulder_asleep_rewakes=");
                    usart2_send_uint(shoulder_asleep_rewakes);
                    usart2_send_string(" elbow_asleep_rewakes=");
                    usart2_send_uint(elbow_asleep_rewakes);
                    // PWR_MGMT_1: 1 = awake as configured, 64 = reset/asleep, 4294967295 = not read yet
                    usart2_send_string(" shoulder_pwr_mgmt_1=");
                    usart2_send_uint(g_pwr_mgmt_1_shoulder);
                    usart2_send_string(" elbow_pwr_mgmt_1=");
                    usart2_send_uint(g_pwr_mgmt_1_elbow);
                    usart2_send_string(" shoulder_power_resets=");
                    usart2_send_uint(g_power_resets_shoulder);
                    usart2_send_string(" elbow_power_resets=");
                    usart2_send_uint(g_power_resets_elbow);
                    // Skipped lines are expected (more output than 115200 baud carries); lost RX bytes should stay 0.
                    usart2_send_string(" uart_skipped_lines=");
                    usart2_send_uint(g_tx.skipped_lines());
                    usart2_send_string(" uart_dropped_bytes=");
                    usart2_send_uint(g_tx.dropped_bytes());
                    usart2_send_string(" uart_rx_overruns=");
                    usart2_send_uint(g_rx_overruns);
                    usart2_send_string(" cal_applied=");
                    usart2_send_uint(g_cal_applied);
                    usart2_send_string(" cal_rejected=");
                    usart2_send_uint(g_cal_rejected);
#if EDGENEURO_DRIVE_SERVOS
                    usart2_send_string(" cal_malformed=");
                    usart2_send_uint(g_cal_parser.rejected());
#endif
                    usart2_send_string("\r\n");
                    }
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

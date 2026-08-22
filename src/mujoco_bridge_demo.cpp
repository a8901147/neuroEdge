// Phase 2, iteration 2: Host-side "twin" of the real firmware control loop
// (firmware/src/phase3_control_loop_main.cpp), replaying a CSV instead of
// real ADC/I2C hardware, emitting one line per sample to stdout for
// tools/mujoco_bridge/run_demo.py to drive a MuJoCo whole-arm+hand
// simulation (tools/mujoco_bridge/arm_hand_scene.xml).
//
// Deliberately bypasses include/edgeneuro/pipeline.hpp's EdgeNeuro<>/
// Pipeline/LdaClassifier path (used by src/main.cpp and src/gui_demo.cpp) --
// that's the Phase 1 engine-generality demo, not the real product's control
// logic. PRD.md Section 3's "Phase 3 control-architecture" note documents
// why the real device uses GripStateMachine (EMG) + ComplementaryFilter
// (IMU) + SlewRateLimiter (smoothing) instead: window-based classification
// has 200ms-class latency, wrong for continuous orientation tracking or
// low-latency grip transitions. This demo shows that same real architecture
// in simulation, not the abandoned classifier path.
//
// Iteration 2 extends iteration 1's single-IMU/wrist-only demo to a full
// shoulder+elbow reach, matching the confirmed real Phase 3 sensor budget of
// 2 MPU6050 IMUs (upper arm + forearm) + 1 MyoWare EMG (PRD.md Section 3,
// 2026-08-22 revision) -- see PRD.md for the full sensor-to-DOF mapping and
// its explicit yaw-unobservable (no magnetometer) limitation. This fully
// replaces iteration 1's behavior in this file (not kept side-by-side): the
// old wearable_1emg_6imu.csv 6-channel format is superseded here by
// wearable_1emg_12imu.csv's 12-channel format, though the old CSV/binary
// pairing still exists for src/main.cpp/tests.
//
// CsvSignalProvider is reused purely as the CSV-iteration mechanism (already
// correct/tested) -- samples are fed directly into GripStateMachine/
// ComplementaryFilter, never into Pipeline/EdgeNeuro<>.

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/slew_rate_limiter.hpp"
#include "edgeneuro/fusion/complementary_filter.hpp"
#include "edgeneuro/providers/csv_signal_provider.hpp"

using namespace edgeneuro;

namespace {

constexpr std::size_t kEmgChannels = 1;
constexpr std::size_t kImuChannels = 12; // imu[0..5]=upper-arm IMU, imu[6..11]=forearm IMU
constexpr std::size_t kMaxSamples = 8192; // covers the generated fixture (2000 rows) with headroom
constexpr float kDt = 0.001f;             // 1kHz stream rate, same convention as src/main.cpp

// data/wearable_1emg_12imu.csv's EMG values (tools/generate_sample_data.py)
// range ~0.0-0.08 at rest and ~0.25-0.95 during contraction bursts (base
// levels 0.03 vs 0.6, +/- noise) -- a completely different scale than real
// hardware's calibrated ADC-count threshold (2037, see
// firmware/src/emg_grip_control_main.cpp). Do NOT reuse that constant here;
// 0.15 sits cleanly between this dataset's rest and contraction ranges.
constexpr float kGripThreshold = 0.15f;
constexpr float kOnDuration = 0.1f;
constexpr float kOffDuration = 0.1f;
// Slower than iteration 1's wrist-only demo (was 5.0): a fast grip-close
// slammed the simulated fingers into the grasp object hard enough to launch
// it off its pedestal -- verified by directly stepping the MuJoCo model.
constexpr float kSlewRate = 1.5f;

using Provider = CsvSignalProvider<float, kEmgChannels, kImuChannels, kMaxSamples>;

} // namespace

int main(int argc, char** argv) {
    const std::string csv_path = argc > 1 ? argv[1] : "data/wearable_1emg_12imu.csv";

    Provider provider(csv_path);
    GripStateMachine<float> grip(kGripThreshold, kOnDuration, kOffDuration);
    SlewRateLimiter<float> setpoint(kSlewRate);
    ComplementaryFilter<float> shoulder_filter(0.98f, kDt); // upper-arm IMU
    ComplementaryFilter<float> elbow_imu_filter(0.98f, kDt); // forearm IMU

    Provider::SampleT sample;
    std::size_t tick = 0;
    bool filters_initialized = false;

    while (provider.next(sample)) {
        const auto start = std::chrono::steady_clock::now();
        ++tick;

        // imu[0..5] = upper-arm IMU [ax,ay,az,gx,gy,gz], imu[6..11] =
        // forearm IMU, same per-IMU order as iteration 1 -- CsvSignalProvider's
        // column-order contract, confirmed against src/main.cpp's own comment.
        const float shoulder_ax = sample.imu[0];
        const float shoulder_ay = sample.imu[1];
        const float shoulder_az = sample.imu[2];
        const float shoulder_gx = sample.imu[3];
        const float shoulder_gy = sample.imu[4];

        const float elbow_ax = sample.imu[6];
        const float elbow_ay = sample.imu[7];
        const float elbow_az = sample.imu[8];
        const float elbow_gx = sample.imu[9];
        const float elbow_gy = sample.imu[10];

        if (!filters_initialized) {
            // skip the cold-start convergence transient
            shoulder_filter.initialize(shoulder_ax, shoulder_ay, shoulder_az);
            elbow_imu_filter.initialize(elbow_ax, elbow_ay, elbow_az);
            filters_initialized = true;
        }
        shoulder_filter.update(shoulder_gx, shoulder_gy, shoulder_ax, shoulder_ay, shoulder_az, kDt);
        elbow_imu_filter.update(elbow_gx, elbow_gy, elbow_ax, elbow_ay, elbow_az, kDt);

        // Elbow flexion = forearm pitch relative to upper-arm pitch. Valid
        // because tools/generate_sample_data.py's wearable_arm_fusion_csv()
        // deliberately builds forearm_pitch = shoulder_pitch + elbow_bend(t)
        // with elbow_bend(t) >= 0 always -- clamped at 0 here to absorb
        // filter noise around the fully-straight pose, not because negative
        // bend is otherwise possible.
        const float elbow_bend = std::max(0.0f, elbow_imu_filter.pitch() - shoulder_filter.pitch());

        grip.update(sample.emg[0], kDt);
        const float grip_setpoint = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDt);

        std::cout << "tick=" << tick
                  << " grip=" << grip_setpoint
                  << " gripping=" << (grip.is_gripping() ? 1 : 0)
                  << " shoulder_pitch=" << shoulder_filter.pitch()
                  << " shoulder_roll=" << shoulder_filter.roll()
                  << " elbow=" << elbow_bend
                  << "\n";
        std::cout.flush(); // required: stdout is fully buffered (not line-buffered) once it's a pipe, not a tty

        // Pace to the 1kHz stream rate the data was generated at, same
        // pattern as src/main.cpp.
        const auto end = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(1000) -
                                     std::chrono::duration_cast<std::chrono::microseconds>(end - start));
    }

    return 0;
}

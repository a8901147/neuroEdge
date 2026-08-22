// Phase 2, iteration 1: Host-side "twin" of the real firmware control loop
// (firmware/src/phase3_control_loop_main.cpp), replaying a CSV instead of
// real ADC/I2C hardware, emitting one line per sample to stdout for
// tools/mujoco_bridge/run_demo.py to drive a MuJoCo Shadow Hand simulation.
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
// Sensor count deliberately matches Phase 1.5's actual validated hardware
// (one real Adafruit MPU-6050 + one MyoWare 2.0, see PRD.md) -- this stays a
// faithful visualization of what the real device's decoded output drives,
// not a speculative extension beyond what's actually been built.
//
// CsvSignalProvider is reused purely as the CSV-iteration mechanism (already
// correct/tested) -- samples are fed directly into GripStateMachine/
// ComplementaryFilter, never into Pipeline/EdgeNeuro<>.

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
constexpr std::size_t kImuChannels = 6;
constexpr std::size_t kMaxSamples = 8192; // covers the generated fixture (2000 rows) with headroom
constexpr float kDt = 0.001f;             // 1kHz stream rate, same convention as src/main.cpp

// data/wearable_1emg_6imu.csv's EMG values (tools/generate_sample_data.py)
// range ~0.0-0.08 at rest and ~0.25-0.95 during contraction bursts (base
// levels 0.03 vs 0.6, +/- noise) -- a completely different scale than real
// hardware's calibrated ADC-count threshold (2037, see
// firmware/src/emg_grip_control_main.cpp). Do NOT reuse that constant here;
// 0.15 sits cleanly between this dataset's rest and contraction ranges.
constexpr float kGripThreshold = 0.15f;
constexpr float kOnDuration = 0.1f;
constexpr float kOffDuration = 0.1f;
constexpr float kSlewRate = 5.0f; // setpoint units/sec, same as firmware/src/emg_grip_control_main.cpp

using Provider = CsvSignalProvider<float, kEmgChannels, kImuChannels, kMaxSamples>;

} // namespace

int main(int argc, char** argv) {
    const std::string csv_path = argc > 1 ? argv[1] : "data/wearable_1emg_6imu.csv";

    Provider provider(csv_path);
    GripStateMachine<float> grip(kGripThreshold, kOnDuration, kOffDuration);
    SlewRateLimiter<float> setpoint(kSlewRate);
    ComplementaryFilter<float> filter(0.98f, kDt);

    Provider::SampleT sample;
    std::size_t tick = 0;
    bool filter_initialized = false;

    while (provider.next(sample)) {
        const auto start = std::chrono::steady_clock::now();
        ++tick;

        // imu columns are [ax,ay,az,gx,gy,gz] -- CsvSignalProvider's column
        // order contract, confirmed against src/main.cpp's own comment.
        const float ax = sample.imu[0];
        const float ay = sample.imu[1];
        const float az = sample.imu[2];
        const float gx = sample.imu[3];
        const float gy = sample.imu[4];

        if (!filter_initialized) {
            filter.initialize(ax, ay, az); // skip the cold-start convergence transient
            filter_initialized = true;
        }
        filter.update(gx, gy, ax, ay, az, kDt);

        grip.update(sample.emg[0], kDt);
        const float grip_setpoint = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDt);

        std::cout << "tick=" << tick
                  << " grip=" << grip_setpoint
                  << " gripping=" << (grip.is_gripping() ? 1 : 0)
                  << " roll=" << filter.roll()
                  << " pitch=" << filter.pitch()
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

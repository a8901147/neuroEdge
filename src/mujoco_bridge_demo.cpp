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
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include "edgeneuro/control/grip_state_machine.hpp"
#include "edgeneuro/control/slew_rate_limiter.hpp"
#include "edgeneuro/fusion/complementary_filter.hpp"
#include "edgeneuro/fusion/tilt_azimuth.hpp"
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

// Real shoulder calibration basis (REST/FORWARD_RAISE/ABDUCTION_LEFT unit
// vectors, this project's real mount -- see
// tools/mujoco_bridge/raw_imu_calibration.json and PRD.md's 2026-09-04/05
// Session Handoff). Replaces ComplementaryFilter::pitch()/roll() for the
// shoulder: that decode has two independent real problems (gyro-
// integration drift with zero corresponding accel change, and an
// accel-only formula that folds back past +-90deg so genuinely different
// poses like forward-raise/backward-extension can decode identically).
// tilt_azimuth.hpp's oblique_decompose fixes both -- see
// tools/mujoco_bridge/run_demo_live.py's matching Python port for the same
// math applied to the live hardware path, and this file's own top comment
// for why this binary needs to mirror that logic instead of just
// firmware's.
//
// raw_imu_calibration.json is a single sitting's 6-pose x 5-repeat capture
// (log_raw_imu.py --repeats 5), averaged per pose -- both more robust than
// a single-shot capture AND has real paired shoulder+elbow readings for
// every pose (an earlier same-day 5-repeat capture's elbow IMU was
// unplugged/silent the whole time, elbow_raw_avg ~(0,0,0) for every pose;
// re-run with the elbow IMU actually connected). Using the SAME session's
// data for both this calibration basis and
// tools/mujoco_bridge/test_imu_to_mujoco.py's fixtures matters: comparing
// against a calibration basis from a DIFFERENT session reintroduces
// exactly the cross-session REST mismatch this project already hit once
// (PRD.md 2026-09-04).
constexpr float kShoulderRefX = 0.96756683f, kShoulderRefY = -0.24536111f, kShoulderRefZ = -0.06010292f;
constexpr float kShoulderFwdX = 0.23614663f, kShoulderFwdY = -0.53617526f, kShoulderFwdZ = 0.81040166f;
constexpr float kShoulderAbdX = 0.11104646f, kShoulderAbdY = -0.91944181f, kShoulderAbdZ = 0.37722067f;

using Provider = CsvSignalProvider<float, kEmgChannels, kImuChannels, kMaxSamples>;

} // namespace

int main(int argc, char** argv) {
    const std::string csv_path = argc > 1 ? argv[1] : "data/wearable_1emg_12imu.csv";

    Provider provider(csv_path);
    GripStateMachine<float> grip(kGripThreshold, kOnDuration, kOffDuration);
    SlewRateLimiter<float> setpoint(kSlewRate);
    ComplementaryFilter<float> shoulder_filter(0.98f, kDt); // upper-arm IMU, old decode kept only for [OLD] comparison output

    const auto shoulder_basis = make_oblique_basis(
        kShoulderRefX, kShoulderRefY, kShoulderRefZ,
        kShoulderFwdX, kShoulderFwdY, kShoulderFwdZ,
        kShoulderAbdX, kShoulderAbdY, kShoulderAbdZ);

    Provider::SampleT sample;
    std::size_t tick = 0;
    bool filters_initialized = false;

    while (provider.next(sample)) {
        const auto start = std::chrono::steady_clock::now();
        ++tick;

        // imu[0..5] = upper-arm IMU [ax,ay,az,gx,gy,gz], imu[6..11] =
        // forearm IMU, same per-IMU order as iteration 1 -- CsvSignalProvider's
        // column-order contract, confirmed against src/main.cpp's own comment.
        const float shoulder_raw_ax = sample.imu[0];
        const float shoulder_raw_ay = sample.imu[1];
        const float shoulder_raw_az = sample.imu[2];
        const float shoulder_raw_gy = sample.imu[4];
        const float shoulder_raw_gz = sample.imu[5];

        const float elbow_raw_ax = sample.imu[6];
        const float elbow_raw_ay = sample.imu[7];
        const float elbow_raw_az = sample.imu[8];

        // 2026-09-03: kept in lockstep with firmware/src/phase3_control_loop_main.cpp's
        // shoulder axis remap and dot-product elbow_bend -- this file had
        // drifted from firmware (no remap at all, plus an Euler-angle-
        // subtraction elbow_bend the firmware itself moved away from for
        // hitting a gimbal-lock-like singularity near real elbow flexion
        // angles) until this pass caught it while building
        // tools/mujoco_bridge/test_imu_to_mujoco.py, which depends on this
        // binary actually reflecting firmware's real decode logic to be a
        // meaningful test. See phase3_control_loop_main.cpp's own remap
        // comment for the full geometric derivation + on-hardware
        // verification this mirrors.
        const float ax = shoulder_raw_ax;
        const float ay = shoulder_raw_az;
        const float az = shoulder_raw_ay;
        const float gx = -shoulder_raw_gy;
        const float gy = shoulder_raw_gz;

        if (!filters_initialized) {
            // skip the cold-start convergence transient
            shoulder_filter.initialize(ax, ay, az);
            filters_initialized = true;
        }
        shoulder_filter.update(gx, gy, ax, ay, az, kDt);

        // Elbow flexion via the raw-vector dot product (angle between the
        // shoulder and elbow readers' own unmapped gravity vectors) -- same
        // formula and same reasoning as firmware's elbow_bend computation:
        // no axis remap needed (the angle between two vectors doesn't care
        // which frame each is expressed in, as long as it's consistent per
        // vector), and no gimbal-lock singularity across the full flexion
        // range, unlike the Euler-subtraction approach this replaced.
        const float shoulder_mag = std::sqrt(shoulder_raw_ax * shoulder_raw_ax +
                                              shoulder_raw_ay * shoulder_raw_ay +
                                              shoulder_raw_az * shoulder_raw_az);

        // Shoulder pitch/roll-equivalent via oblique_decompose_scaled, on
        // the RAW (pre-remap) shoulder axes -- shoulder_basis's calibration
        // readings are themselves raw sensor axes (same convention as
        // tools/mujoco_bridge/log_raw_imu.py's capture), not the ax/ay/az
        // remap above (that remap is specific to ComplementaryFilter's own
        // axis convention, kept above only for the [OLD] comparison
        // output). oblique_decompose_scaled normalizes internally, so the
        // raw (unnormalized) axes are passed directly -- see its own
        // comment for why a plain oblique_decompose scaled by the
        // calibration's own tilt overshoots for off-axis poses (e.g. a
        // real ~16deg ELBOW_FLEXION drift that overshot to ~35deg).
        // Guarded the same way as elbow_bend below: a stationary/
        // disconnected reader can report near-zero magnitude, which
        // oblique_decompose_scaled's own internal normalize would blow up.
        float pitch_equiv = 0.0f, roll_equiv = 0.0f;
        if (shoulder_mag > 0.1f) {
            const auto coeffs = oblique_decompose_scaled(shoulder_basis, shoulder_raw_ax, shoulder_raw_ay, shoulder_raw_az);
            pitch_equiv = coeffs.fwd;
            roll_equiv = coeffs.abd;
        }

        const float elbow_mag = std::sqrt(elbow_raw_ax * elbow_raw_ax +
                                           elbow_raw_ay * elbow_raw_ay +
                                           elbow_raw_az * elbow_raw_az);
        float elbow_bend = 0.0f;
        if (shoulder_mag > 0.1f && elbow_mag > 0.1f) {
            const float dot = shoulder_raw_ax * elbow_raw_ax + shoulder_raw_ay * elbow_raw_ay +
                               shoulder_raw_az * elbow_raw_az;
            float cos_angle = dot / (shoulder_mag * elbow_mag);
            if (cos_angle > 1.0f) cos_angle = 1.0f;
            if (cos_angle < -1.0f) cos_angle = -1.0f;
            elbow_bend = std::acos(cos_angle);
        }

        grip.update(sample.emg[0], kDt);
        const float grip_setpoint = setpoint.update(grip.is_gripping() ? 1.0f : 0.0f, kDt);

        // shoulder_pitch=/shoulder_roll= (ComplementaryFilter, unchanged)
        // must stay exactly as-is: tools/mujoco_bridge/run_demo.py is a
        // SEPARATE consumer of this same binary that replays the fully
        // synthetic data/wearable_1emg_12imu.csv (tools/generate_sample_data.py),
        // which has no relationship to this real mount's calibration basis
        // -- feeding it through oblique_decompose would produce meaningless
        // values. pitch_equiv=/roll_equiv= are ADDED fields, read only by
        // tools/mujoco_bridge/test_imu_to_mujoco.py's real-hardware-fixture
        // path (see this file's top comment).
        std::cout << "tick=" << tick
                  << " grip=" << grip_setpoint
                  << " gripping=" << (grip.is_gripping() ? 1 : 0)
                  << " shoulder_pitch=" << shoulder_filter.pitch()
                  << " shoulder_roll=" << shoulder_filter.roll()
                  << " elbow=" << elbow_bend
                  << " pitch_equiv=" << pitch_equiv
                  << " roll_equiv=" << roll_equiv
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

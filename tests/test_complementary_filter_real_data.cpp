// Validates ComplementaryFilter against real EMG-EPN-612 IMU data -- not
// just the known-physics unit tests in test_complementary_filter.cpp, but
// an actual comparison against ground truth captured on real hardware.
//
// The Myo armband computes its own onboard sensor-fusion orientation
// (quaternion) using its accelerometer + gyroscope + magnetometer. That
// quaternion is exactly the kind of independent ground truth our
// accel+gyro-only complementary filter can be checked against: convert the
// Myo's quaternion to roll/pitch with the standard aerospace formula, and
// compare to what our own filter converges to when fed the SAME
// accelerometer/gyroscope stream.
//
// Units, confirmed (not guessed) two ways before writing this test:
//   1. The official Myo BLE protocol header (myohw.h, Thalmic Labs) documents
//      accelerometer in g, gyroscope in deg/s, orientation as a unit
//      quaternion -- each scaled by a fixed constant in the raw SDK format.
//   2. The actual values in this dataset's JSON were checked against that:
//      accelerometer vector magnitude at rest ~= 1.0 (consistent with g),
//      quaternion component sum-of-squares ~= 1.0 (consistent with an
//      already-normalized unit quaternion), gyroscope values are single
//      digits (consistent with deg/s during a slow hand motion, not raw
//      SDK counts which would be in the thousands).
// So: gyroscope needs deg/s -> rad/s conversion before reaching
// ComplementaryFilter (which takes rad/s); accelerometer's g scale is fine
// as-is (ComplementaryFilter only uses it via atan2, so absolute scale
// doesn't matter).
//
// This test uses the first recording in user1.json (992 EMG-rate rows,
// gesture "noGesture" -- i.e. the subject's arm is essentially stationary
// throughout), read through the same zero-order-hold-resampled CSV as
// test_real_data_integration.cpp. That resampling repeats each native IMU
// reading across multiple EMG-rate rows; running the filter at that finer
// rate is a reasonable stand-in for running at the IMU's native ~50Hz (the
// physical integral of a piecewise-constant gyro rate is the same either
// way) and is also what real Phase 3 firmware would do -- see README's
// "Real-dataset compatibility" section.
//
// Recording duration isn't stored per-sample in this dataset's JSON, only
// documented as "approximately five seconds" in the source paper -- dt is
// derived from that approximation (5s / 992 rows), same class of
// documented approximation tools/convert_epn612.py already makes for the
// EMG/IMU sample-rate mismatch. Getting this exactly right isn't the point
// here: the point is checking the filter converges toward the real
// hardware's own orientation estimate, not deriving a precise physical
// timebase.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <string>

#include "edgeneuro/fusion/complementary_filter.hpp"
#include "edgeneuro/providers/csv_signal_provider.hpp"
#include "edgeneuro/sample.hpp"

using Catch::Approx;
using edgeneuro::ComplementaryFilter;
using edgeneuro::CsvSignalProvider;

namespace {
const std::string kRealDataPath = std::string(EDGENEURO_DATA_DIR) + "/raw/epn612_user1.csv";

// Standard aerospace (ZYX) quaternion -> roll/pitch, used only to read out
// the Myo's own onboard fusion result as ground truth for this test -- not
// part of the product engine, which never has a magnetometer-fused
// quaternion available on the real MPU6050 target.
float quaternion_roll(float w, float x, float y, float z) {
    return std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
}

float quaternion_pitch(float w, float x, float y, float z) {
    float sin_pitch = 2.0f * (w * y - z * x);
    sin_pitch = std::max(-1.0f, std::min(1.0f, sin_pitch));
    return std::asin(sin_pitch);
}
} // namespace

TEST_CASE("ComplementaryFilter tracks the Myo's own onboard orientation on a real static recording", "[fusion][real_data]") {
    if (!std::filesystem::exists(kRealDataPath)) {
        SKIP("Real dataset not found at " << kRealDataPath
             << " -- see README 'Real-dataset compatibility' to download + convert it locally.");
    }

    constexpr std::size_t kEmgChannels = 8;
    constexpr std::size_t kImuChannels = 10; // acc(3) + gyro(3) + quat(4), see convert_epn612.py
    // First recording in user1.json ("noGesture", 992 EMG-rate rows) -- see
    // this file's header comment for why this exact count.
    constexpr std::size_t kRows = 992;
    constexpr float kAssumedDurationSeconds = 5.0f; // see header comment: dataset paper's approximation
    constexpr float kDt = kAssumedDurationSeconds / static_cast<float>(kRows);

    // IMU column indices within Sample::imu, matching convert_epn612.py's header order.
    constexpr std::size_t kAccX = 0, kAccY = 1, kAccZ = 2;
    constexpr std::size_t kGyroX = 3, kGyroY = 4;
    constexpr std::size_t kQuatW = 6, kQuatX = 7, kQuatY = 8, kQuatZ = 9;

    CsvSignalProvider<float, kEmgChannels, kImuChannels, kRows> provider(kRealDataPath);
    ComplementaryFilter<float> filter(0.98f, kDt);

    edgeneuro::Sample<float, kEmgChannels, kImuChannels> sample;
    std::size_t rows_read = 0;
    float max_abs_roll = 0.0f;
    float final_quat_roll = 0.0f;
    float final_quat_pitch = 0.0f;

    while (provider.next(sample)) {
        const float deg_to_rad = std::numbers::pi_v<float> / 180.0f;
        const float gyro_x_rad = sample.imu[kGyroX] * deg_to_rad;
        const float gyro_y_rad = sample.imu[kGyroY] * deg_to_rad;

        if (rows_read == 0) {
            // Skip the cold-start convergence transient (see
            // ComplementaryFilter::initialize's doc comment) -- irrelevant
            // to what this test checks, which is steady-state agreement
            // with the Myo's own fusion output, not startup behavior.
            filter.initialize(sample.imu[kAccX], sample.imu[kAccY], sample.imu[kAccZ]);
        }
        filter.update(gyro_x_rad, gyro_y_rad, sample.imu[kAccX], sample.imu[kAccY], sample.imu[kAccZ]);

        REQUIRE(std::isfinite(filter.roll()));
        REQUIRE(std::isfinite(filter.pitch()));
        max_abs_roll = std::max(max_abs_roll, std::abs(filter.roll()));

        final_quat_roll = quaternion_roll(sample.imu[kQuatW], sample.imu[kQuatX], sample.imu[kQuatY], sample.imu[kQuatZ]);
        final_quat_pitch = quaternion_pitch(sample.imu[kQuatW], sample.imu[kQuatX], sample.imu[kQuatY], sample.imu[kQuatZ]);

        ++rows_read;
    }
    REQUIRE(rows_read == kRows);

    // A stationary arm shouldn't produce a wildly swinging roll estimate --
    // catches gross sign/feedback errors without requiring exact numeric
    // agreement anywhere but the final comparison below. (Empirically
    // ~1.20 rad for this recording -- gravity isn't aligned with any single
    // sensor axis given how the armband sits on a forearm.)
    REQUIRE(max_abs_roll < 2.0f);

    // The real check: after processing the whole recording, our accel+gyro
    // -only estimate should closely match the Myo's own onboard fusion
    // (accel+gyro+magnetometer) for this same stream. Verified empirically
    // in Python against this exact file before writing this assertion:
    // final diff ~1.4e-4 rad (roll) / ~2.8e-4 rad (pitch). 0.01 rad
    // (~0.6 degrees) leaves generous margin for float vs double and any
    // remaining rounding in the CSV's 6-decimal text encoding.
    REQUIRE(filter.roll() == Approx(final_quat_roll).margin(0.01));
    REQUIRE(filter.pitch() == Approx(final_quat_pitch).margin(0.01));
}

#pragma once

#include <array>
#include <cstdint>

namespace edgeneuro::mearm::calibration_link {

// The MEArm calibration sent over UART (2026-10-03, the user's choice "B"): run_demo_live.py sends it instead of the
// calibration being compiled in (no re-flash after a re-calibration). One line:
//   C<v0>,<v1>,...,<v19>,<checksum>\n
// 20 values, each round(value * 1e6) as a signed decimal integer:
//   0-2 HANG, 3-5 FORWARD, 6-8 LEFT, 9-11 RIGHT (raw gravity vectors, g), 12 zero_elbow (rad),
//   13 has_reach (0 or 1e6), 14-16 base reach LEFT, 17-19 base reach RIGHT (raw vectors)
// checksum = the sum of the 20 integers mod 1000000007 (made non-negative). Fixed-point integers so the firmware needs
// no text-to-float parsing. Parser: one byte at a time (the firmware's RX path), no allocation; a message counts only
// if it is complete (exactly 20 values + checksum) and its checksum matches -- then the caller still validates the
// calibration itself (pathb::Calibration::make) before using it. Python encoder: mearm_calibration_link.py.
constexpr int kFieldCount = 20;
constexpr long long kChecksumModulus = 1000000007LL;
constexpr int kMaxDigits = 12;     // a longer number is garbage, not a value (and would overflow)

struct Values {
    std::array<float, 3> hang, forward, left, right;
    float zero_elbow;
    bool has_reach;
    std::array<float, 3> reach_left, reach_right;
};

class Parser {
public:
    // Feed one received byte; true when it completed a valid message (then values() holds it).
    bool feed(uint8_t b) {
        if (b == static_cast<uint8_t>('C')) {               // a new message, even mid-message: never glue two together
            start();
            return false;
        }
        if (!active_) return false;                          // bytes of other commands (T..., R): not ours
        if (b == static_cast<uint8_t>('\r')) return false;
        if (b == static_cast<uint8_t>('-')) {
            if (digits_ != 0 || negative_) return reject();
            negative_ = true;
            return false;
        }
        if (b >= static_cast<uint8_t>('0') && b <= static_cast<uint8_t>('9')) {
            if (++digits_ > kMaxDigits) return reject();
            current_ = current_ * 10 + static_cast<long long>(b - static_cast<uint8_t>('0'));
            return false;
        }
        if (b == static_cast<uint8_t>(',') || b == static_cast<uint8_t>('\n')) {
            if (digits_ == 0 || count_ >= kFieldCount + 1) return reject();
            fields_[static_cast<unsigned>(count_++)] = negative_ ? -current_ : current_;
            current_ = 0;
            digits_ = 0;
            negative_ = false;
            if (b == static_cast<uint8_t>(',')) return false;
            active_ = false;
            if (count_ != kFieldCount + 1) return reject();
            long long sum = 0;
            for (int i = 0; i < kFieldCount; ++i) sum += fields_[static_cast<unsigned>(i)];
            sum %= kChecksumModulus;
            if (sum < 0) sum += kChecksumModulus;
            if (sum != fields_[kFieldCount]) return reject();
            decode();
            return true;
        }
        return reject();                                     // anything else inside a message
    }

    const Values& values() const { return values_; }
    unsigned rejected() const { return rejected_; }

private:
    void start() {
        active_ = true;
        count_ = 0;
        current_ = 0;
        digits_ = 0;
        negative_ = false;
    }
    bool reject() {
        active_ = false;
        ++rejected_;
        return false;
    }
    float f(int i) const {   // exact decimal -> nearest float (one double division; once per message)
        return static_cast<float>(static_cast<double>(fields_[static_cast<unsigned>(i)]) / 1e6);
    }
    void decode() {
        for (int k = 0; k < 3; ++k) {
            values_.hang[static_cast<unsigned>(k)] = f(0 + k);
            values_.forward[static_cast<unsigned>(k)] = f(3 + k);
            values_.left[static_cast<unsigned>(k)] = f(6 + k);
            values_.right[static_cast<unsigned>(k)] = f(9 + k);
            values_.reach_left[static_cast<unsigned>(k)] = f(14 + k);
            values_.reach_right[static_cast<unsigned>(k)] = f(17 + k);
        }
        values_.zero_elbow = f(12);
        values_.has_reach = fields_[13] != 0;
    }

    bool active_ = false;
    int count_ = 0;
    long long current_ = 0;
    int digits_ = 0;
    bool negative_ = false;
    std::array<long long, kFieldCount + 1> fields_{};
    Values values_{};
    unsigned rejected_ = 0;
};

}  // namespace edgeneuro::mearm::calibration_link

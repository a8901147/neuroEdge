#pragma once

#include <cstdint>

namespace edgeneuro::emg_threshold_link {

// The EMG threshold line from run_demo_live.py's send_emg_threshold(): "T<on>\n" (one threshold, the format since
// 2026-09-09) or, since 2026-10-04, "T<on>,<release>\n" (grip above <on>, let go only below <release> --
// GripStateMachine::set_thresholds). Parsed one received byte at a time (the firmware's RX path); '\r' is ignored, a new
// 'T' restarts, anything else malformed drops the line -- nothing is applied half-way. Bytes outside a T line belong to
// other commands (R, the C calibration message) and are ignored.
constexpr unsigned kMaxDigits = 5;   // the ADC is 12-bit (<= 4095): anything longer is garbage

class Parser {
public:
    // true when `b` completed a valid line (then on()/release() hold it)
    bool feed(uint8_t b) {
        if (b == static_cast<uint8_t>('T')) {
            active_ = true;
            count_ = 0;
            digits_ = 0;
            value_ = 0u;
            return false;
        }
        if (!active_ || b == static_cast<uint8_t>('\r')) return false;
        if (b >= static_cast<uint8_t>('0') && b <= static_cast<uint8_t>('9')) {
            if (++digits_ > kMaxDigits) return drop();
            value_ = value_ * 10u + static_cast<uint32_t>(b - static_cast<uint8_t>('0'));
            return false;
        }
        if (b == static_cast<uint8_t>(',') || b == static_cast<uint8_t>('\n')) {
            if (digits_ == 0 || count_ >= 2) return drop();
            fields_[count_++] = value_;
            value_ = 0u;
            digits_ = 0;
            if (b == static_cast<uint8_t>(',')) return false;
            active_ = false;
            on_ = fields_[0];
            release_ = count_ == 2 ? fields_[1] : fields_[0];
            return true;
        }
        return drop();
    }

    uint32_t on() const { return on_; }
    uint32_t release() const { return release_; }

private:
    bool drop() {
        active_ = false;
        return false;
    }

    bool active_ = false;
    unsigned count_ = 0;
    unsigned digits_ = 0;
    uint32_t value_ = 0u;
    uint32_t fields_[2]{};
    uint32_t on_ = 0u;
    uint32_t release_ = 0u;
};

}  // namespace edgeneuro::emg_threshold_link

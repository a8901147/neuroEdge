#pragma once

#include <cstddef>
#include <cstdint>

namespace edgeneuro {

// Fixed-size byte queue for sending UART lines without stopping the control loop (2026-10-03, measured on the real
// board: busy-waiting on each ~346-byte line took ~30 ms, so the 1 kHz loop ran at 291 ticks/s and every tick-based
// time was ~3.4x short). The loop pushes a line's bytes and hands the UART one byte whenever it is free. A line that
// does not fit is skipped WHOLE (begin_line), never sent half; a refused single push is counted. Single producer and
// single consumer in the same thread (no interrupts involved). No allocation.
template <std::size_t N>
class TxRing {
    static_assert(N > 0, "TxRing needs a capacity");

public:
    bool push(uint8_t b) noexcept {
        if (count_ == N) {
            ++dropped_bytes_;
            return false;
        }
        buf_[head_] = b;
        head_ = (head_ + 1u) % N;
        ++count_;
        return true;
    }

    bool pop(uint8_t& out) noexcept {
        if (count_ == 0u) return false;
        out = buf_[tail_];
        tail_ = (tail_ + 1u) % N;
        --count_;
        return true;
    }

    // true if `max_len` bytes fit now; otherwise the whole line is skipped (counted) and nothing should be pushed.
    bool begin_line(std::size_t max_len) noexcept {
        if (free_space() >= max_len) return true;
        ++skipped_lines_;
        return false;
    }

    std::size_t free_space() const noexcept { return N - count_; }
    uint32_t dropped_bytes() const noexcept { return dropped_bytes_; }
    uint32_t skipped_lines() const noexcept { return skipped_lines_; }

private:
    uint8_t buf_[N]{};
    std::size_t head_ = 0u, tail_ = 0u, count_ = 0u;
    uint32_t dropped_bytes_ = 0u, skipped_lines_ = 0u;
};

}  // namespace edgeneuro

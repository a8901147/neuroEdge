#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace edgeneuro {

// Single-producer/single-consumer lock-free ring buffer backed entirely by
// std::array. No heap allocation at any point in its lifetime. Capacity
// must be a power of two so the index wrap uses a bitmask instead of a
// modulo (branch-free on the hot path).
template <typename T, std::size_t Capacity>
class RingBuffer {
    static_assert(Capacity >= 2, "RingBuffer capacity must be at least 2");
    static_assert((Capacity & (Capacity - 1)) == 0, "RingBuffer capacity must be a power of two");

public:
    constexpr RingBuffer() noexcept = default;

    // Producer side. Returns false if the buffer is full (never blocks/allocates).
    bool push(const T& value) noexcept {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        const std::size_t next = (head + 1) & kMask;
        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // full
        }
        buffer_[head] = value;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Consumer side. Returns false if the buffer is empty.
    bool pop(T& out) noexcept {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // empty
        }
        out = buffer_[tail];
        tail_.store((tail + 1) & kMask, std::memory_order_release);
        return true;
    }

    std::size_t size() const noexcept {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & kMask;
    }

    bool empty() const noexcept { return size() == 0; }
    bool full() const noexcept { return size() == Capacity - 1; }

    static constexpr std::size_t capacity() noexcept { return Capacity; }

private:
    static constexpr std::size_t kMask = Capacity - 1;

    alignas(64) std::array<T, Capacity> buffer_{};
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

} // namespace edgeneuro

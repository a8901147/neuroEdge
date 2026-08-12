#pragma once

#include <array>
#include <cstddef>

namespace edgeneuro {

// Fixed-capacity bump allocator: hands out sequentially-aligned slices of
// a static buffer, never frees individual allocations (whole-arena reset
// only). Used by Target's operator new/delete override
// (firmware/src/no_heap_guard_target.cpp) as the fallback backing store
// for the rare legitimate allocation, since there's no newlib heap
// (_sbrk) on that build. Pure logic, no hardware dependency — unlike the
// GPIO-touching code around it in firmware/, this is testable on Host
// like everything else in this directory.
template <std::size_t Capacity>
class BumpAllocator {
public:
    // Returns nullptr if the request doesn't fit, exactly like a
    // allocation-failure path operator new is expected to handle (throw
    // bad_alloc) — never asserts/aborts itself.
    void* allocate(std::size_t size) noexcept {
        const std::size_t aligned_used = align_up(used_);
        if (aligned_used + size > Capacity) {
            return nullptr;
        }
        used_ = aligned_used + size;
        return &arena_[aligned_used];
    }

    void reset() noexcept { used_ = 0; }
    std::size_t used() const noexcept { return used_; }
    static constexpr std::size_t capacity() noexcept { return Capacity; }

private:
    static constexpr std::size_t kAlign = alignof(std::max_align_t);

    static std::size_t align_up(std::size_t value) noexcept {
        return (value + kAlign - 1) & ~(kAlign - 1);
    }

    alignas(kAlign) std::array<unsigned char, Capacity> arena_{};
    std::size_t used_{0};
};

} // namespace edgeneuro

// Bare-metal operator new/delete override backing edgeneuro::NoHeapGuard,
// target-side counterpart to src/no_heap_guard.cpp (Host). Reuses
// include/edgeneuro/no_heap_guard.hpp UNMODIFIED (it's pure std::atomic,
// no OS dependency) -- only the "what happens on violation" reaction
// differs, because there's no abort()/stderr here: no OS, and pulling in
// newlib's syscall stubs (_write, _exit, _kill, ...) just to print an
// error message is exactly the kind of libc surface area this project
// avoids on target. A fast LED blink is the target-side equivalent of
// "abort and make it loud" -- distinguishable by eye from Stage 0's slow
// ~1Hz blink.
//
// There is no real heap backing this (no _sbrk, no newlib malloc
// configured) -- on this firmware there is categorically no legitimate
// reason for operator new to ever be called, armed or not. The tiny
// static arena below exists only so an unarmed call (which Host's
// contract allows) doesn't corrupt memory; it is not meant to be
// exercised by anything in this project's current firmware.

#include <cstddef>
#include <cstdint>
#include <new>

#include "stm32f4xx.h"

#include "edgeneuro/no_heap_guard.hpp"

namespace {

constexpr unsigned kLedPin = 13u;

[[noreturn]] void signal_violation_and_halt() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3u << (kLedPin * 2u));
    GPIOC->MODER |= (1u << (kLedPin * 2u));
    while (true) {
        GPIOC->ODR ^= (1u << kLedPin);
        for (uint32_t i = 0; i < 50000u; ++i) {
            __asm__ volatile("nop"); // ~16x faster toggle than Stage 0's blink -- visually distinct
        }
    }
}

// Fallback arena for the (unexpected, never-armed) case: satisfies the
// operator-new contract without needing newlib's heap.
constexpr std::size_t kArenaSize = 1024;
alignas(alignof(std::max_align_t)) unsigned char g_arena[kArenaSize];
std::size_t g_arena_used = 0;

void* bump_allocate(std::size_t size) noexcept {
    if (g_arena_used + size > kArenaSize) {
        return nullptr;
    }
    void* ptr = &g_arena[g_arena_used];
    g_arena_used += size;
    return ptr;
}

void record_allocation() noexcept {
    edgeneuro::NoHeapGuard::allocation_count().fetch_add(1, std::memory_order_relaxed);
    if (edgeneuro::NoHeapGuard::armed().load(std::memory_order_relaxed)) {
        signal_violation_and_halt();
    }
}

} // namespace

void* operator new(std::size_t size) {
    record_allocation();
    if (void* ptr = bump_allocate(size)) {
        return ptr;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
    record_allocation();
    if (void* ptr = bump_allocate(size)) {
        return ptr;
    }
    throw std::bad_alloc();
}

// Bump arena is never freed piecemeal -- deletes are no-ops. Fine: nothing
// in this firmware is expected to allocate at all, so nothing frees either.
void operator delete(void*) noexcept {}
void operator delete(void*, std::size_t) noexcept {}
void operator delete[](void*) noexcept {}
void operator delete[](void*, std::size_t) noexcept {}

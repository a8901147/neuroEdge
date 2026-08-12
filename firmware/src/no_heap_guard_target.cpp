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
// edgeneuro::BumpAllocator below (pure logic, Host-tested in
// tests/test_bump_allocator.cpp) exists only so an unarmed call (which
// Host's contract allows) doesn't corrupt memory; it is not meant to be
// exercised by anything in this project's current firmware. Exhausting
// it halts the same way an armed violation does, rather than throwing
// std::bad_alloc() -- the linker script discards .ARM.exidx (no unwind
// tables), so a real C++ exception here would be undefined behavior, not
// a graceful failure.

#include <cstddef>

#include "stm32f4xx.h"

#include "edgeneuro/bump_allocator.hpp"
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

edgeneuro::BumpAllocator<1024> g_arena;

void record_allocation() noexcept {
    edgeneuro::NoHeapGuard::allocation_count().fetch_add(1, std::memory_order_relaxed);
    if (edgeneuro::NoHeapGuard::armed().load(std::memory_order_relaxed)) {
        signal_violation_and_halt();
    }
}

void* allocate_or_halt(std::size_t size) noexcept {
    record_allocation();
    if (void* ptr = g_arena.allocate(size)) {
        return ptr;
    }
    signal_violation_and_halt(); // arena exhausted: no unwind tables to throw bad_alloc into
}

} // namespace

void* operator new(std::size_t size) { return allocate_or_halt(size); }
void* operator new[](std::size_t size) { return allocate_or_halt(size); }

// Bump arena is never freed piecemeal -- deletes are no-ops. Fine: nothing
// in this firmware is expected to allocate at all, so nothing frees either.
void operator delete(void*) noexcept {}
void operator delete(void*, std::size_t) noexcept {}
void operator delete[](void*) noexcept {}
void operator delete[](void*, std::size_t) noexcept {}

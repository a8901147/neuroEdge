#pragma once

#include <atomic>
#include <cstddef>

namespace edgeneuro {

// Verifies malloc_count == 0 across a hot-loop run. Only meaningful in the
// `debug-heapguard` / `release-bench` CMake presets, where src/no_heap_guard.cpp
// is compiled in and overrides the global operator new/delete to count (and
// optionally abort on) every heap allocation.
//
// Deliberately NOT linked into the ASan/UBSan/TSan presets: those sanitizers
// install their own operator new/delete interceptors for shadow-memory
// bookkeeping, and layering a second global override on top of them is
// undefined/fragile. Sanitizer builds verify absence of UB and data races;
// this guard verifies absence of allocation. Two separate concerns, two
// separate binaries — see CMakePresets.json.
class NoHeapGuard {
public:
    // RAII: arms the guard for the lifetime of the scope. While armed, any
    // heap allocation aborts the process immediately (see no_heap_guard.cpp).
    NoHeapGuard() noexcept { armed().store(true, std::memory_order_relaxed); }
    ~NoHeapGuard() noexcept { armed().store(false, std::memory_order_relaxed); }

    NoHeapGuard(const NoHeapGuard&) = delete;
    NoHeapGuard& operator=(const NoHeapGuard&) = delete;

    static void reset_count() noexcept { allocation_count().store(0, std::memory_order_relaxed); }
    static std::size_t count() noexcept { return allocation_count().load(std::memory_order_relaxed); }

    static std::atomic<std::size_t>& allocation_count() noexcept {
        static std::atomic<std::size_t> count{0};
        return count;
    }

    static std::atomic<bool>& armed() noexcept {
        static std::atomic<bool> armed_flag{false};
        return armed_flag;
    }
};

} // namespace edgeneuro

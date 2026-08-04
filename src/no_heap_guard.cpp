// Global operator new/delete overrides backing edgeneuro::NoHeapGuard.
// This translation unit is only compiled into the debug-heapguard and
// release-bench CMake presets (see CMakePresets.json / CMakeLists.txt) —
// never alongside ASan/UBSan/TSan, which install their own allocator hooks.

#include <cstdio>
#include <cstdlib>
#include <new>

#include "edgeneuro/no_heap_guard.hpp"

namespace {

void record_allocation() noexcept {
    edgeneuro::NoHeapGuard::allocation_count().fetch_add(1, std::memory_order_relaxed);
    if (edgeneuro::NoHeapGuard::armed().load(std::memory_order_relaxed)) {
        std::fputs("NoHeapGuard: heap allocation detected inside an armed hot loop\n", stderr);
        std::abort();
    }
}

} // namespace

void* operator new(std::size_t size) {
    record_allocation();
    if (void* ptr = std::malloc(size)) {
        return ptr;
    }
    throw std::bad_alloc();
}

void* operator new[](std::size_t size) {
    record_allocation();
    if (void* ptr = std::malloc(size)) {
        return ptr;
    }
    throw std::bad_alloc();
}

void operator delete(void* ptr) noexcept { std::free(ptr); }
void operator delete(void* ptr, std::size_t) noexcept { std::free(ptr); }
void operator delete[](void* ptr) noexcept { std::free(ptr); }
void operator delete[](void* ptr, std::size_t) noexcept { std::free(ptr); }

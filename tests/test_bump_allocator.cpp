#include <catch2/catch_test_macros.hpp>
#include <cstdint>

#include "edgeneuro/bump_allocator.hpp"

using edgeneuro::BumpAllocator;

TEST_CASE("BumpAllocator starts empty", "[bump_allocator]") {
    BumpAllocator<64> a;
    REQUIRE(a.used() == 0);
    REQUIRE(a.capacity() == 64);
}

TEST_CASE("BumpAllocator serves an in-capacity request", "[bump_allocator]") {
    BumpAllocator<64> a;
    void* p = a.allocate(16);
    REQUIRE(p != nullptr);
    REQUIRE(a.used() >= 16);
}

TEST_CASE("BumpAllocator rejects a request that doesn't fit", "[bump_allocator]") {
    BumpAllocator<16> a;
    REQUIRE(a.allocate(32) == nullptr);
    REQUIRE(a.used() == 0); // a failed request must not consume capacity
}

TEST_CASE("BumpAllocator hands out non-overlapping regions", "[bump_allocator]") {
    BumpAllocator<256> a;
    auto* p1 = static_cast<unsigned char*>(a.allocate(10));
    auto* p2 = static_cast<unsigned char*>(a.allocate(10));
    REQUIRE(p1 != nullptr);
    REQUIRE(p2 != nullptr);
    REQUIRE(p2 >= p1 + 10); // p2 must not start before p1's request ends
}

TEST_CASE("BumpAllocator aligns every allocation to alignof(std::max_align_t)", "[bump_allocator]") {
    // Odd sizes are exactly the case that would break a naive
    // "just advance by size" bump pointer without rounding up.
    BumpAllocator<256> a;
    a.allocate(1);
    void* p = a.allocate(1);
    REQUIRE(p != nullptr);
    REQUIRE(reinterpret_cast<std::uintptr_t>(p) % alignof(std::max_align_t) == 0);
}

TEST_CASE("BumpAllocator exhausts capacity exactly, then rejects further requests", "[bump_allocator]") {
    BumpAllocator<32> a;
    // Drain it in aligned-size chunks so we hit capacity precisely rather
    // than leaving an alignment-padding remainder.
    while (a.allocate(alignof(std::max_align_t)) != nullptr) {
    }
    REQUIRE(a.allocate(1) == nullptr);
}

TEST_CASE("BumpAllocator::reset reclaims all capacity", "[bump_allocator]") {
    BumpAllocator<32> a;
    REQUIRE(a.allocate(32) != nullptr);
    REQUIRE(a.allocate(1) == nullptr); // exhausted

    a.reset();
    REQUIRE(a.used() == 0);
    REQUIRE(a.allocate(32) != nullptr); // full capacity available again
}

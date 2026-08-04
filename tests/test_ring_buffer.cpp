#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/ring_buffer.hpp"

using edgeneuro::RingBuffer;

TEST_CASE("RingBuffer starts empty", "[ring_buffer]") {
    RingBuffer<int, 8> rb;
    REQUIRE(rb.empty());
    REQUIRE(rb.size() == 0);
    REQUIRE(rb.capacity() == 8);

    int out = 0;
    REQUIRE_FALSE(rb.pop(out));
}

TEST_CASE("RingBuffer push/pop preserves FIFO order", "[ring_buffer]") {
    RingBuffer<int, 8> rb;
    for (int i = 0; i < 5; ++i) {
        REQUIRE(rb.push(i));
    }
    REQUIRE(rb.size() == 5);

    for (int i = 0; i < 5; ++i) {
        int out = -1;
        REQUIRE(rb.pop(out));
        REQUIRE(out == i);
    }
    REQUIRE(rb.empty());
}

TEST_CASE("RingBuffer rejects push when full", "[ring_buffer]") {
    RingBuffer<int, 4> rb; // usable capacity = Capacity - 1 = 3
    REQUIRE(rb.push(1));
    REQUIRE(rb.push(2));
    REQUIRE(rb.push(3));
    REQUIRE(rb.full());
    REQUIRE_FALSE(rb.push(4));

    int out = 0;
    REQUIRE(rb.pop(out));
    REQUIRE(out == 1);
    REQUIRE(rb.push(4));
}

TEST_CASE("RingBuffer wraps around the backing array correctly", "[ring_buffer]") {
    RingBuffer<int, 4> rb;
    for (int cycle = 0; cycle < 10; ++cycle) {
        REQUIRE(rb.push(cycle));
        REQUIRE(rb.push(cycle * 100));
        int a = 0, b = 0;
        REQUIRE(rb.pop(a));
        REQUIRE(rb.pop(b));
        REQUIRE(a == cycle);
        REQUIRE(b == cycle * 100);
    }
    REQUIRE(rb.empty());
}

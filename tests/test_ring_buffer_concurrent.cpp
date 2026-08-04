// Simulates the Phase 3 hand-off this RingBuffer is actually built for:
// one thread stands in for the STM32 Timer/DMA ISR (a fast, jittery
// producer), another stands in for the main loop that runs the DSP
// pipeline (a slower, independently-paced consumer). Run under the
// `sanitize-tsan` preset to prove the lock-free SPSC protocol is free of
// data races before any real hardware exists.

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

#include "edgeneuro/ring_buffer.hpp"

using edgeneuro::RingBuffer;

TEST_CASE("RingBuffer survives a real async producer/consumer race", "[ring_buffer][concurrent]") {
    constexpr int kSamples = 20000;
    RingBuffer<int, 1024> rb;

    std::atomic<bool> producer_done{false};
    std::vector<long long> consumed;
    consumed.reserve(kSamples);

    std::thread producer([&] {
        for (int i = 0; i < kSamples; ++i) {
            while (!rb.push(i)) {
                std::this_thread::yield(); // buffer full: back off, exactly like a busy main loop would
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::thread consumer([&] {
        int value = 0;
        while (!producer_done.load(std::memory_order_acquire) || !rb.empty()) {
            if (rb.pop(value)) {
                consumed.push_back(value);
            }
        }
    });

    producer.join();
    consumer.join();

    REQUIRE(consumed.size() == static_cast<std::size_t>(kSamples));
    for (int i = 0; i < kSamples; ++i) {
        REQUIRE(consumed[static_cast<std::size_t>(i)] == i); // strict FIFO ordering under real concurrency
    }
}

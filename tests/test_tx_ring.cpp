// TxRing: the byte queue that lets phase3_control_loop send its UART lines WITHOUT stopping the main loop (2026-10-03,
// measured on the real board: the busy-wait send of each ~346-byte line took ~30 ms, the 1 kHz loop ran at 291
// ticks/s, so every tick-based time -- servo ramps, filter dt -- was ~3.4x short). The loop now only queues bytes and
// feeds one to the UART whenever it is free; a whole line that does not fit is skipped instead of blocking.
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/control/tx_ring.hpp"

using Ring = edgeneuro::TxRing<8>;

TEST_CASE("TxRing: bytes come out in the order they went in", "[tx_ring]") {
    Ring r;
    for (uint8_t b : {1, 2, 3}) REQUIRE(r.push(b));
    uint8_t out = 0;
    for (uint8_t want : {1, 2, 3}) {
        REQUIRE(r.pop(out));
        REQUIRE(out == want);
    }
    REQUIRE_FALSE(r.pop(out));
}

TEST_CASE("TxRing: holds exactly its capacity, a push into a full ring is refused and counted", "[tx_ring]") {
    Ring r;
    for (uint8_t i = 0; i < 8; ++i) REQUIRE(r.push(i));
    REQUIRE(r.free_space() == 0u);
    REQUIRE_FALSE(r.push(99));
    REQUIRE(r.dropped_bytes() == 1u);
    uint8_t out = 0;
    REQUIRE(r.pop(out));
    REQUIRE(out == 0u);                                  // the refused byte did not overwrite anything
}

TEST_CASE("TxRing: keeps order across the wrap-around, many times over", "[tx_ring]") {
    Ring r;
    uint8_t next_in = 0, next_out = 0, out = 0;
    for (int round = 0; round < 100; ++round) {
        for (int i = 0; i < 5; ++i) REQUIRE(r.push(next_in++));
        for (int i = 0; i < 5; ++i) {
            REQUIRE(r.pop(out));
            REQUIRE(out == next_out++);
        }
    }
    REQUIRE(r.free_space() == 8u);
}

TEST_CASE("TxRing: free_space tracks pushes and pops", "[tx_ring]") {
    Ring r;
    REQUIRE(r.free_space() == 8u);
    r.push(1);
    r.push(2);
    REQUIRE(r.free_space() == 6u);
    uint8_t out = 0;
    r.pop(out);
    REQUIRE(r.free_space() == 7u);
}

TEST_CASE("TxRing: a line is started only if all of it fits -- otherwise it is skipped whole and counted",
          "[tx_ring]") {
    Ring r;
    REQUIRE(r.begin_line(5));                             // fits: 8 free
    for (uint8_t i = 0; i < 5; ++i) r.push(i);
    REQUIRE_FALSE(r.begin_line(4));                       // 3 free: skip, never a half line on the wire
    REQUIRE(r.skipped_lines() == 1u);
    REQUIRE(r.free_space() == 3u);                        // nothing was queued for the skipped line
    REQUIRE(r.begin_line(3));
}

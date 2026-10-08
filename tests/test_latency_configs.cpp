// The two latency configurations shared by the host Google Benchmark (benchmarks/bench_pipeline.cpp) and the
// on-target cycle-count benchmark (firmware/src/pipeline_latency_main.cpp), so both measure exactly the same engines;
// plus the cycle statistics the firmware reports.
#include <cstdint>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/bench/latency_configs.hpp"
#include "edgeneuro/no_heap_guard.hpp"

namespace B = edgeneuro::bench;

TEST_CASE("both latency engines run without allocating and classify once per window", "[bench]") {
    auto wearable = B::make_wearable_engine();
    auto hd = B::make_hd_engine();
    std::size_t out = 0;
    std::size_t wearable_results = 0, hd_results = 0;
    edgeneuro::NoHeapGuard::reset_count();
    {
        edgeneuro::NoHeapGuard guard;
        for (std::size_t i = 0; i < 10 * B::kWindow; ++i) {
            wearable.tick(out);
            wearable_results += wearable.has_result() ? 1u : 0u;
            hd.tick(out);
            hd_results += hd.has_result() ? 1u : 0u;
        }
    }
    REQUIRE(edgeneuro::NoHeapGuard::count() == 0u);
    REQUIRE(wearable_results == 10u);
    REQUIRE(hd_results == 10u);
}

TEST_CASE("the configurations are the ones the benchmarks describe", "[bench]") {
    STATIC_REQUIRE(B::kWearableEmgChannels == 1u);
    STATIC_REQUIRE(B::kWearableImuChannels == 6u);
    STATIC_REQUIRE(B::WearableEngine::TotalChannels == 7u);
    STATIC_REQUIRE(B::kHdEmgChannels == 32u);
    STATIC_REQUIRE(B::kHdImuChannels == 0u);
    STATIC_REQUIRE(B::HdEngine::TotalChannels == 32u);
}

TEST_CASE("cycle statistics: min, max and mean of the recorded samples", "[bench]") {
    B::CycleStats s;
    REQUIRE(s.count() == 0u);
    REQUIRE(s.mean() == 0.0);
    for (std::uint32_t c : {120u, 100u, 400u, 180u}) s.add(c);
    REQUIRE(s.count() == 4u);
    REQUIRE(s.min() == 100u);
    REQUIRE(s.max() == 400u);
    REQUIRE(s.mean() == 200.0);
}

TEST_CASE("cycle statistics do not overflow over a long run", "[bench]") {
    B::CycleStats s;
    for (int i = 0; i < 100000; ++i) s.add(4000000000u);   // close to UINT32_MAX each
    REQUIRE(s.mean() == 4000000000.0);
}

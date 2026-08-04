#pragma once

#include <array>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

#include "edgeneuro/sample.hpp"

namespace edgeneuro {

// Host-side SignalProvider: replays a CSV file (Ninapro-style EMG slices or
// synchronized EMG+IMU sequences) as a deterministic 1kHz sample stream.
//
// All parsing (file I/O, string splitting, std::string allocation) happens
// once in the constructor, which runs before the hot loop starts and is
// therefore exempt from the zero-allocation guarantee. next() — the only
// method called per tick — does nothing but index into a pre-parsed
// std::array, so it is noexcept and allocation-free.
//
// Expected CSV layout: one row per sample, columns in order
// [emg_0..emg_{EmgChannels-1}, imu_0..imu_{ImuChannels-1}]. An optional
// non-numeric header row is detected and skipped automatically.
template <typename ValueType, std::size_t EmgChannels, std::size_t ImuChannels, std::size_t MaxSamples>
class CsvSignalProvider {
public:
    using SampleT = Sample<ValueType, EmgChannels, ImuChannels>;
    static constexpr std::size_t kColumns = EmgChannels + ImuChannels;

    explicit CsvSignalProvider(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("CsvSignalProvider: failed to open file: " + path);
        }

        std::string line;
        while (count_ < MaxSamples && std::getline(file, line)) {
            SampleT sample;
            if (parse_row(line, sample)) {
                samples_[count_++] = sample;
            }
            // Rows that fail to parse (e.g. a header line) are silently skipped.
        }
    }

    // Hot-path: zero allocation, noexcept, O(1).
    bool next(SampleT& out) noexcept {
        if (read_index_ >= count_) {
            return false;
        }
        out = samples_[read_index_++];
        return true;
    }

    void rewind() noexcept { read_index_ = 0; }

    std::size_t size() const noexcept { return count_; }
    std::size_t remaining() const noexcept { return count_ - read_index_; }

private:
    // Uses strtof rather than std::from_chars: Apple's libc++ still gates the
    // floating-point from_chars overloads behind a deployment-target
    // availability check, making them unreliable across toolchains. strtof
    // is plain C, has no such caveat, and this parser only ever runs at
    // setup time (never in the hot loop), so its heap-free-ness doesn't matter.
    bool parse_row(const std::string& line, SampleT& out) const {
        std::array<ValueType, kColumns> values{};
        std::size_t col = 0;
        std::size_t start = 0;
        bool reached_end = false; // true once the last-parsed field had no trailing comma

        while (col < kColumns) {
            const std::size_t comma = line.find(',', start);
            const std::size_t field_end = (comma == std::string::npos) ? line.size() : comma;

            char* end_ptr = nullptr;
            const float parsed = std::strtof(line.c_str() + start, &end_ptr);
            const std::size_t consumed = static_cast<std::size_t>(end_ptr - (line.c_str() + start));
            if (consumed == 0 || start + consumed != field_end) {
                return false; // non-numeric field or trailing garbage: treat row as a header/comment
            }
            values[col++] = static_cast<ValueType>(parsed);

            if (comma == std::string::npos) {
                reached_end = true;
                break;
            }
            start = comma + 1;
        }

        // col == kColumns alone isn't enough: a row with MORE than kColumns
        // fields would also satisfy it (the loop simply stops reading once
        // enough columns are collected), silently truncating extra data
        // instead of rejecting the row. reached_end distinguishes "exactly
        // kColumns fields" from "at least kColumns fields".
        if (col != kColumns || !reached_end) {
            return false; // malformed row: wrong column count
        }

        for (std::size_t c = 0; c < EmgChannels; ++c) {
            out.emg[c] = values[c];
        }
        for (std::size_t c = 0; c < ImuChannels; ++c) {
            out.imu[c] = values[EmgChannels + c];
        }
        return true;
    }

    std::array<SampleT, MaxSamples> samples_{};
    std::size_t count_{0};
    std::size_t read_index_{0};
};

} // namespace edgeneuro

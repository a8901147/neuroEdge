#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "edgeneuro/providers/csv_signal_provider.hpp"

using Catch::Approx;
using edgeneuro::CsvSignalProvider;
using edgeneuro::Sample;

namespace {

// RAII temp file so each test cleans up after itself regardless of outcome.
// Uses mkstemp (POSIX, host-only) rather than std::tmpnam, which is
// deprecated for the classic TOCTOU reasons.
class TempCsv {
public:
    explicit TempCsv(const std::string& content) {
        const std::string tmpl = (std::filesystem::temp_directory_path() / "edgeneuro_test_XXXXXX").string();
        std::vector<char> buf(tmpl.begin(), tmpl.end());
        buf.push_back('\0');
        const int fd = mkstemp(buf.data());
        path_ = buf.data();

        std::ofstream file(path_);
        file << content;
        file.close();
        ::close(fd);
    }
    ~TempCsv() { std::remove(path_.c_str()); }
    const std::string& path() const { return path_; }

private:
    std::string path_;
};

} // namespace

TEST_CASE("CsvSignalProvider parses a 1-EMG + 6-IMU stream and skips the header", "[csv_provider]") {
    TempCsv csv(
        "emg0,imu0,imu1,imu2,imu3,imu4,imu5\n"
        "0.1,1,2,3,4,5,6\n"
        "0.2,1.1,2.1,3.1,4.1,5.1,6.1\n");

    CsvSignalProvider<float, 1, 6, 16> provider(csv.path());
    REQUIRE(provider.size() == 2);

    Sample<float, 1, 6> sample;
    REQUIRE(provider.next(sample));
    REQUIRE(sample.emg[0] == Approx(0.1f));
    REQUIRE(sample.imu[0] == Approx(1.0f));
    REQUIRE(sample.imu[5] == Approx(6.0f));

    REQUIRE(provider.next(sample));
    REQUIRE(sample.emg[0] == Approx(0.2f));

    REQUIRE_FALSE(provider.next(sample)); // exhausted
}

TEST_CASE("CsvSignalProvider handles headerless 32-channel HD-sEMG rows", "[csv_provider]") {
    std::string content;
    for (int row = 0; row < 3; ++row) {
        for (int ch = 0; ch < 32; ++ch) {
            content += std::to_string(row * 32 + ch);
            content += (ch == 31 ? '\n' : ',');
        }
    }
    TempCsv csv(content);

    CsvSignalProvider<float, 32, 0, 8> provider(csv.path());
    REQUIRE(provider.size() == 3);

    Sample<float, 32, 0> sample;
    REQUIRE(provider.next(sample));
    REQUIRE(sample.emg[0] == Approx(0.0f));
    REQUIRE(sample.emg[31] == Approx(31.0f));
}

TEST_CASE("CsvSignalProvider throws when the file does not exist", "[csv_provider]") {
    REQUIRE_THROWS_AS(
        (CsvSignalProvider<float, 1, 1, 8>("/nonexistent/path/does_not_exist.csv")),
        std::runtime_error);
}

TEST_CASE("CsvSignalProvider skips malformed rows (wrong column count)", "[csv_provider]") {
    // Row 2 is missing a column, row 3 has an extra one — both should be
    // silently skipped, same as a header row, not crash or miscount.
    TempCsv csv("1,2\n3\n5,6,7\n8,9\n");
    CsvSignalProvider<float, 1, 1, 8> provider(csv.path());
    REQUIRE(provider.size() == 2); // only "1,2" and "8,9" are well-formed

    Sample<float, 1, 1> sample;
    REQUIRE(provider.next(sample));
    REQUIRE(sample.emg[0] == Approx(1.0f));
    REQUIRE(provider.next(sample));
    REQUIRE(sample.emg[0] == Approx(8.0f));
}

TEST_CASE("CsvSignalProvider stops loading at MaxSamples capacity", "[csv_provider]") {
    TempCsv csv("1,2\n3,4\n5,6\n7,8\n");
    CsvSignalProvider<float, 1, 1, 2> provider(csv.path()); // capacity 2, file has 4 rows
    REQUIRE(provider.size() == 2);
}

TEST_CASE("CsvSignalProvider::rewind replays the stream from the start", "[csv_provider]") {
    TempCsv csv("1,2\n3,4\n");
    CsvSignalProvider<float, 1, 1, 8> provider(csv.path());

    Sample<float, 1, 1> sample;
    REQUIRE(provider.next(sample));
    REQUIRE(provider.next(sample));
    REQUIRE_FALSE(provider.next(sample));

    provider.rewind();
    REQUIRE(provider.next(sample));
    REQUIRE(sample.emg[0] == Approx(1.0f));
}

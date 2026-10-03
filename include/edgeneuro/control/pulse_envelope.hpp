#pragma once

#include <cmath>
#include <cstddef>

namespace edgeneuro {

struct Pulses {
    unsigned shoulder;
    unsigned elbow;
};

// Keeps a (shoulder, elbow) pair of servo pulse widths inside a MEASURED envelope: one elbow window
// [lo, hi] per measured shoulder position (breakpoints ascending). Deliberately coarse and conservative:
//   * exactly at a breakpoint its own window applies;
//   * BETWEEN two breakpoints the two windows are INTERSECTED -- never interpolated, so nothing is assumed about
//     places nobody measured;
//   * the shoulder may not leave [first breakpoint, last breakpoint] (no claim about anything beyond).
// A NaN input -- or an empty table / empty window, which means the table is broken -- gives the REST pose, which the
// data is tested to contain. Results are whole microseconds (rounded to nearest). Non-owning: the arrays must outlive it.
// Python reference: tools/gen_mearm_envelope.py (clamp); tests/test_pulse_envelope.cpp checks both, and the raw data.
class PulseEnvelope {
public:
    constexpr PulseEnvelope(const unsigned* shoulder_us, const unsigned* elbow_lo_us, const unsigned* elbow_hi_us, std::size_t n,
                  unsigned rest_shoulder_us, unsigned rest_elbow_us) noexcept
        : s_(shoulder_us), lo_(elbow_lo_us), hi_(elbow_hi_us), n_(n),
          rest_{rest_shoulder_us, rest_elbow_us} {}

    Pulses clamp(float shoulder_us, float elbow_us) const noexcept {
        if (n_ == 0u || std::isnan(shoulder_us) || std::isnan(elbow_us)) {
            return rest_;
        }
        const float s_lo = static_cast<float>(s_[0]);
        const float s_hi = static_cast<float>(s_[n_ - 1u]);
        const float sc = shoulder_us < s_lo ? s_lo : (shoulder_us > s_hi ? s_hi : shoulder_us);   // +-inf clamp
        const unsigned s = static_cast<unsigned>(sc + 0.5f);

        unsigned lo = 0u, hi = 0u;
        bool found = false;
        for (std::size_t i = 0; i < n_ && !found; ++i) {
            if (s == s_[i]) {
                lo = lo_[i];
                hi = hi_[i];
                found = true;
            }
        }
        for (std::size_t i = 0; i + 1u < n_ && !found; ++i) {
            if (s_[i] < s && s < s_[i + 1u]) {
                lo = lo_[i] > lo_[i + 1u] ? lo_[i] : lo_[i + 1u];       // the more conservative of the two
                hi = hi_[i] < hi_[i + 1u] ? hi_[i] : hi_[i + 1u];
                found = true;
            }
        }
        if (!found || lo > hi) {
            return rest_;
        }
        const float flo = static_cast<float>(lo), fhi = static_cast<float>(hi);
        const float ec = elbow_us < flo ? flo : (elbow_us > fhi ? fhi : elbow_us);
        return {s, static_cast<unsigned>(ec + 0.5f)};
    }

    // The measured shoulder range (first / last breakpoint); the rest pulse if the table is empty.
    constexpr unsigned shoulder_min() const noexcept { return n_ ? s_[0] : rest_.shoulder; }
    constexpr unsigned shoulder_max() const noexcept { return n_ ? s_[n_ - 1u] : rest_.shoulder; }

private:
    const unsigned* s_;
    const unsigned* lo_;
    const unsigned* hi_;
    std::size_t n_;
    Pulses rest_;
};

} // namespace edgeneuro

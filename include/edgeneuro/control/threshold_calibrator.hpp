#pragma once

#include <limits>

namespace edgeneuro {

// Turns a two-phase "relax, then contract" calibration sequence into a
// GripStateMachine threshold, instead of a hardcoded magic number that
// only holds for one specific session's electrode placement, skin
// contact, and gain-pot setting (see PRD.md Stage 5a: 1220 happened to
// work, but only by luck against that one real-hardware run).
//
// Caller contract: during the "relax" phase, call observe_relaxed() with
// every sample while the user is deliberately at rest; during the
// "contract" phase (after switching phases), call observe_contracted()
// with every sample while the user is deliberately holding a contraction.
// Order matters -- this class does not detect which phase it's in, the
// caller decides that from wall-clock timing (see emg_grip_control_main.cpp).
template <typename ValueType>
class ThresholdCalibrator {
public:
    void observe_relaxed(ValueType sample) noexcept {
        has_relaxed_ = true;
        if (sample > relaxed_max_) {
            relaxed_max_ = sample;
        }
    }

    void observe_contracted(ValueType sample) noexcept {
        has_contracted_ = true;
        if (sample < contracted_min_) {
            contracted_min_ = sample;
        }
    }

    // Midpoint between the loudest relaxed noise seen and the quietest
    // contracted signal seen -- equal margin against a false grip trigger
    // from noise and a missed real contraction, not biased toward either
    // failure mode without a specific reason to.
    ValueType threshold() const noexcept { return (relaxed_max_ + contracted_min_) / ValueType{2}; }

    // False means calibration didn't see a clean separation (e.g. no
    // observe_*() calls were made, bad electrode contact, or the
    // "contraction" phase didn't actually contain a real contraction) --
    // the caller must not trust threshold() in that case. Requires both
    // phases to have actually been observed at least once, not just a
    // relaxed_max_/contracted_min_ ordering check: the sentinel initial
    // values happen to already satisfy that ordering before any real
    // observation is ever made, which would otherwise report a fake pass.
    bool is_valid() const noexcept {
        return has_relaxed_ && has_contracted_ && contracted_min_ > relaxed_max_;
    }

    // Exposed for diagnostics -- e.g. printing why is_valid() came back
    // false (which of the two phases was the problem) instead of just
    // reporting a bare pass/fail.
    ValueType relaxed_max() const noexcept { return relaxed_max_; }
    ValueType contracted_min() const noexcept { return contracted_min_; }

    void reset() noexcept {
        relaxed_max_ = ValueType{0};
        contracted_min_ = std::numeric_limits<ValueType>::max();
        has_relaxed_ = false;
        has_contracted_ = false;
    }

private:
    ValueType relaxed_max_{0};
    ValueType contracted_min_{std::numeric_limits<ValueType>::max()};
    bool has_relaxed_{false};
    bool has_contracted_{false};
};

} // namespace edgeneuro

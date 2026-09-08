#pragma once

namespace edgeneuro {

// Threshold + hysteresis (debounce) state machine: turns a single EMG
// envelope channel into a discrete grip/release decision.
//
// Deliberately NOT a Filter/Feature/Classifier (concepts.hpp) and does NOT
// go through Pipeline's window/feature/classify path -- that path only
// produces a result once every WindowSize samples (e.g. 200ms at
// WindowSize=200 @ 1kHz), which is the wrong latency budget for "did the
// user just start/stop gripping." This is the same reasoning that keeps
// ComplementaryFilter (see fusion/complementary_filter.hpp) out of
// Pipeline too; see PRD.md Section 3's Phase 3 control-architecture note.
//
// Also deliberately simpler than a trained classifier (LdaClassifier):
// with a single EMG channel, there is only one dimension of information
// available ("how hard is this one muscle contracting right now"), so a
// multi-class classifier buys nothing a threshold doesn't already give.
// This is standard practice for single-channel myoelectric control, not a
// project-specific shortcut.
//
// Intended input is MyoWare 2.0's ENV (envelope) output, not RAW -- ENV is
// already rectified + low-pass filtered in analog hardware before it ever
// reaches the ADC, so no further filtering/feature extraction is needed
// upstream of this class.
//
// Caller contract: call update() once per fresh envelope sample with dt
// (seconds) since the last call. envelope's unit is whatever the caller's
// ADC produces (raw counts are fine) as long as `threshold` is in the same
// unit -- only ever compared directly, never rescaled.
template <typename ValueType>
class GripStateMachine {
public:
    enum class State { Released, Gripping };

    // threshold: envelope value above which the muscle counts as
    // contracting.
    // on_duration/off_duration: how long the envelope must stay
    // continuously above/below threshold before the state actually flips
    // -- this is what rejects a brief noise spike or a momentary twitch
    // from triggering a grip/release, not just the threshold itself.
    GripStateMachine(ValueType threshold, ValueType on_duration, ValueType off_duration) noexcept
        : threshold_(threshold), on_duration_(on_duration), off_duration_(off_duration) {}

    // Returns true only on the update() call where the state actually
    // changes (a transition edge) -- false on every other call, including
    // while continuously held above/below threshold. Check state() (or
    // is_gripping()) for the current state at any time; check this return
    // value only if the caller specifically needs to react to the moment
    // of transition (e.g. to fire an actuator command once, not every tick).
    bool update(ValueType envelope, ValueType dt) noexcept {
        if (envelope > threshold_) {
            above_time_ += dt;
            below_time_ = ValueType{0};
        } else {
            below_time_ += dt;
            above_time_ = ValueType{0};
        }

        if (state_ == State::Released && above_time_ >= on_duration_) {
            state_ = State::Gripping;
            return true;
        }
        if (state_ == State::Gripping && below_time_ >= off_duration_) {
            state_ = State::Released;
            return true;
        }
        return false;
    }

    State state() const noexcept { return state_; }
    bool is_gripping() const noexcept { return state_ == State::Gripping; }

    // Live threshold update -- added 2026-09-09 so a host-computed
    // calibration (relax/clench captured and averaged on the Python side,
    // from the envelope this class already streams every tick regardless)
    // can be applied without reconstructing this object or restarting the
    // main loop. Does NOT reset above_time_/below_time_/state_: an
    // in-progress hold shouldn't be discarded just because a fresher
    // threshold arrived -- only the comparison in the NEXT update() call
    // uses the new value.
    void set_threshold(ValueType threshold) noexcept { threshold_ = threshold; }
    ValueType threshold() const noexcept { return threshold_; }

    void reset() noexcept {
        above_time_ = ValueType{0};
        below_time_ = ValueType{0};
        state_ = State::Released;
    }

private:
    ValueType threshold_;
    ValueType on_duration_;
    ValueType off_duration_;
    ValueType above_time_{0};
    ValueType below_time_{0};
    State state_{State::Released};
};

} // namespace edgeneuro

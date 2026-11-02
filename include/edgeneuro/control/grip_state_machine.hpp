#pragma once

namespace edgeneuro {

// Threshold state machine with debounce and hysteresis: turns one EMG envelope channel into a grip/release decision.
//
// Not a Filter/Feature/Classifier (concepts.hpp) and not part of Pipeline: Pipeline decides once per window (200 ms at
// WindowSize=200, 1 kHz), too slow for "did the grip just start". With one EMG channel there is one dimension of
// information, so a threshold is the standard single-site myoelectric switch; a trained classifier adds nothing.
//
// Input: the MyoWare 2.0 ENV output (rectified and low-passed in hardware). The debounce needs the envelope above
// (below) the threshold WITHOUT interruption for on_duration (off_duration): one sample on the wrong side restarts the
// count. So the caller smooths first (phase3_control_loop_main.cpp: an EMA, kEmgSmoothingAlpha); ENV alone chattered
// near the threshold on the hardware.
//
// Call update() once per sample with dt in seconds. The envelope and the thresholds share one unit (raw ADC counts
// are fine); they are only compared, never rescaled.
template <typename ValueType>
class GripStateMachine {
public:
    enum class State { Released, Gripping };

    // threshold: envelope level above which the muscle counts as contracting (also the release level until
    // set_thresholds() sets a lower one). on_duration/off_duration: how long the envelope must stay above/below it
    // before the state flips, which rejects a noise spike or a brief twitch.
    GripStateMachine(ValueType threshold, ValueType on_duration, ValueType off_duration) noexcept
        : threshold_(threshold), release_(threshold), on_duration_(on_duration), off_duration_(off_duration) {}

    // Returns true only on the call where the state changes; state()/is_gripping() give the current state.
    bool update(ValueType envelope, ValueType dt) noexcept {
        // Hysteresis: while gripping, only falling below the release threshold counts as letting go.
        const ValueType level = state_ == State::Gripping ? release_ : threshold_;
        if (envelope > level) {
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

    // Live update from the host's calibration. The state and the running debounce times are kept: an ongoing grip is
    // not dropped because a new threshold arrived; the next update() compares against the new values.
    void set_threshold(ValueType threshold) noexcept { set_thresholds(threshold, threshold); }
    // Grip above `threshold`, release only below `release` (capped at `threshold`): a grip held more gently while the
    // arm moves can dip under the grip threshold without letting go.
    void set_thresholds(ValueType threshold, ValueType release) noexcept {
        threshold_ = threshold;
        release_ = release < threshold ? release : threshold;
    }
    ValueType threshold() const noexcept { return threshold_; }
    ValueType release_threshold() const noexcept { return release_; }

    void reset() noexcept {
        above_time_ = ValueType{0};
        below_time_ = ValueType{0};
        state_ = State::Released;
    }

private:
    ValueType threshold_;
    ValueType release_;
    ValueType on_duration_;
    ValueType off_duration_;
    ValueType above_time_{0};
    ValueType below_time_{0};
    State state_{State::Released};
};

} // namespace edgeneuro

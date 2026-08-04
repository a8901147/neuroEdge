#pragma once

namespace edgeneuro {

// Second-order IIR filter (biquad), Direct Form II implementation:
// only two state registers regardless of coefficient values, which keeps
// the per-channel footprint constant and avoids any heap use.
//
//   w[n]  = x[n] - a1*w[n-1] - a2*w[n-2]
//   y[n]  = b0*w[n] + b1*w[n-1] + b2*w[n-2]
//
// Coefficients are precomputed offline (e.g. Butterworth band-pass design
// for EMG conditioning) and injected at construction; a0 is assumed
// normalized to 1.
template <typename ValueType>
class IirFilter {
public:
    // Defaults to the identity filter (b0=1, all else 0) so arrays of
    // per-channel filters can be value-initialized before their real
    // coefficients are assigned.
    constexpr IirFilter() noexcept : IirFilter(ValueType{1}, ValueType{0}, ValueType{0}, ValueType{0}, ValueType{0}) {}

    constexpr IirFilter(ValueType b0, ValueType b1, ValueType b2, ValueType a1, ValueType a2) noexcept
        : b0_(b0), b1_(b1), b2_(b2), a1_(a1), a2_(a2) {}

    ValueType process(ValueType input) noexcept {
        const ValueType w = input - a1_ * w1_ - a2_ * w2_;
        const ValueType output = b0_ * w + b1_ * w1_ + b2_ * w2_;
        w2_ = w1_;
        w1_ = w;
        return output;
    }

    void reset() noexcept {
        w1_ = ValueType{0};
        w2_ = ValueType{0};
    }

private:
    ValueType b0_;
    ValueType b1_;
    ValueType b2_;
    ValueType a1_;
    ValueType a2_;
    ValueType w1_{0};
    ValueType w2_{0};
};

} // namespace edgeneuro

#pragma once
/// @file biquad.hpp
/// @brief Second-order IIR sections for gyro and accelerometer filtering, and their design.
///
/// A biquad realizes
///
///   H(z) = (b0 + b1 z⁻¹ + b2 z⁻²) / (1 + a1 z⁻¹ + a2 z⁻²)
///
/// in Direct Form I:
///
///   y[n] = b0 x[n] + b1 x[n−1] + b2 x[n−2] − a1 y[n−1] − a2 y[n−2]
///
/// Direct Form I keeps only past inputs and outputs as state. Every stored
/// value is a signal sample, so the state never leaves the signal's range
/// (narrow scalar types do not overflow inside the filter), and a change of
/// coefficients takes effect without a jump in the state (the RPM notch
/// retunes every tick).
///
/// Design functions (bilinear transform with prewarping, so each design is
/// exact at its specified frequency):
///  - butterworth_lowpass(fc, fs): 2nd-order Butterworth, |H(fc)| = 1/√2.
///  - first_order_lowpass(fc, fs): the first-order ("PT1") low-pass,
///    |H(fc)| = 1/√2, as a biquad with b2 = a2 = 0.
///  - notch(f0, q, fs): the Audio EQ Cookbook notch. H(f0) = 0 exactly; unity
///    gain at DC and at fs/2. Its −3 dB width is (fs/π)·atan(sin ω0 / 2q),
///    which tends to f0/q for f0 ≪ fs.
/// Each takes frequencies in Hz and requires 0 < f < fs/2. A violated
/// precondition is a configuration fault: it asserts, and in a release build
/// the function returns the pass-through section instead of a broken filter.
///
/// Analysis (design time, evaluated in double): response() is the complex
/// gain at a frequency, group_delay() the group delay in seconds, from the
/// closed form −d∠H/dω = Re(Σ k b_k e^{−jωk} / Σ b_k e^{−jωk}) − (same for a).
///
/// Model
/// -----
///   Biquad<T>
///     c_       : BiquadCoefficients<T> [owned] — the section
///     x1_, x2_ : T — inputs x[n−1], x[n−2]
///     y1_, y2_ : T — outputs y[n−1], y[n−2]
///
/// Invariants
/// ----------
///   I1: biquad_coefficients_valid(c_): finite, poles strictly inside the unit
///       circle                                          — constrains c_
///   I2: the history is finite                           — constrains x1_..y2_
///
/// Validity envelope and non-goals: a linear time-invariant section between
/// retunes. It does not model the sensor (aliasing, quantization, sample
/// jitter) and does not choose its own frequencies.
///
/// Scalar types: any type with +, −, ×, ÷, <, sin, cos, abs, isfinite, and
/// construction from double (design and steady-state start); analysis also
/// needs static_cast<double>. Tested in double and float; 16-bit types are
/// characterized, not supported (see the usage page).
///
/// Resources: header-only, no allocation, no exceptions. update() runs a fixed
/// 9 operations (5 ×, 4 ±) on finite input.

#include <cassert>
#include <cmath>
#include <complex>
#include <numbers>

namespace branes::reflex::dsp {

/// Coefficients of H(z) = (b0 + b1 z⁻¹ + b2 z⁻²) / (1 + a1 z⁻¹ + a2 z⁻²).
/// The default is the pass-through section H(z) = 1.
template <typename T>
struct BiquadCoefficients {
    T b0{1};
    T b1{0};
    T b2{0};
    T a1{0};
    T a2{0};
};

/// True if `c` is finite and its poles lie strictly inside the unit circle
/// (the stability triangle |a2| < 1, |a1| < 1 + a2). Implies 1 + a1 + a2 > 0,
/// so the DC gain is defined. Invariant I1 of Biquad.
template <typename T>
[[nodiscard]] bool biquad_coefficients_valid(const BiquadCoefficients<T>& c) noexcept {
    using std::abs;
    using std::isfinite;
    const T one(1);
    return isfinite(c.b0) && isfinite(c.b1) && isfinite(c.b2) && isfinite(c.a1) && isfinite(c.a2) && abs(c.a2) < one &&
           abs(c.a1) < one + c.a2;
}

/// Gain at DC, H(1) = (b0 + b1 + b2) / (1 + a1 + a2). Pre: valid(c).
template <typename T>
[[nodiscard]] T dc_gain(const BiquadCoefficients<T>& c) noexcept {
    return (c.b0 + c.b1 + c.b2) / (T(1) + c.a1 + c.a2);
}

/// True if 0 < f < fs/2 and both are finite: a design frequency the bilinear
/// transform can place.
template <typename T>
[[nodiscard]] bool design_frequency_valid(T f, T fs) noexcept {
    using std::isfinite;
    return isfinite(f) && isfinite(fs) && T(0) < f && T(2) * f < fs;
}

/// 2nd-order Butterworth low-pass with its −3 dB point at fc [Hz].
/// Pre: design_frequency_valid(fc, fs).
template <typename T>
[[nodiscard]] BiquadCoefficients<T> butterworth_lowpass(T fc, T fs) noexcept {
    if (!design_frequency_valid(fc, fs)) {
        assert(false && "butterworth_lowpass: need 0 < fc < fs/2");  // Pre
        return {};
    }
    using std::cos;
    using std::sin;
    const T w = T(std::numbers::pi) * fc / fs;
    const T k = sin(w) / cos(w);  // prewarped tan(π fc / fs)
    const T kk = k * k;
    const T sqrt2 = T(std::numbers::sqrt2);
    const T norm = T(1) / (T(1) + sqrt2 * k + kk);
    const T b0 = kk * norm;
    return {b0, T(2) * b0, b0, T(2) * (kk - T(1)) * norm, (T(1) - sqrt2 * k + kk) * norm};
}

/// First-order low-pass with its −3 dB point at fc [Hz] (b2 = a2 = 0).
/// Pre: design_frequency_valid(fc, fs).
template <typename T>
[[nodiscard]] BiquadCoefficients<T> first_order_lowpass(T fc, T fs) noexcept {
    if (!design_frequency_valid(fc, fs)) {
        assert(false && "first_order_lowpass: need 0 < fc < fs/2");  // Pre
        return {};
    }
    using std::cos;
    using std::sin;
    const T w = T(std::numbers::pi) * fc / fs;
    const T k = sin(w) / cos(w);
    const T b0 = k / (T(1) + k);
    return {b0, b0, T(0), (k - T(1)) / (T(1) + k), T(0)};
}

/// Notch at f0 [Hz] with quality factor q (Audio EQ Cookbook, α = sin ω0 / 2q).
/// Pre: design_frequency_valid(f0, fs), q > 0 and finite.
template <typename T>
[[nodiscard]] BiquadCoefficients<T> notch(T f0, T q, T fs) noexcept {
    using std::isfinite;
    if (!design_frequency_valid(f0, fs) || !isfinite(q) || !(T(0) < q)) {
        assert(false && "notch: need 0 < f0 < fs/2 and q > 0");  // Pre
        return {};
    }
    using std::cos;
    using std::sin;
    const T w0 = T(2) * T(std::numbers::pi) * f0 / fs;
    const T alpha = sin(w0) / (T(2) * q);
    const T inv_a0 = T(1) / (T(1) + alpha);
    const T b1 = -(T(2) * cos(w0)) * inv_a0;
    return {inv_a0, b1, inv_a0, b1, (T(1) - alpha) * inv_a0};
}

/// Complex gain H(e^{jω}) at f [Hz], ω = 2π f / fs, evaluated in double.
template <typename T>
[[nodiscard]] std::complex<double> response(const BiquadCoefficients<T>& c, double f, double fs) noexcept {
    const std::complex<double> z1 = std::polar(1.0, -2.0 * std::numbers::pi * f / fs);
    const std::complex<double> z2 = z1 * z1;
    const auto d = [](T v) { return static_cast<double>(v); };
    return (d(c.b0) + d(c.b1) * z1 + d(c.b2) * z2) / (1.0 + d(c.a1) * z1 + d(c.a2) * z2);
}

/// Group delay −d∠H/dω at f [Hz], in seconds, from the closed form (not a
/// numerical difference). Pre: H(f) ≠ 0 (not at a notch's centre).
template <typename T>
[[nodiscard]] double group_delay(const BiquadCoefficients<T>& c, double f, double fs) noexcept {
    const std::complex<double> z1 = std::polar(1.0, -2.0 * std::numbers::pi * f / fs);
    const std::complex<double> z2 = z1 * z1;
    const auto d = [](T v) { return static_cast<double>(v); };
    // For P(z⁻¹) = Σ p_k z⁻ᵏ, −d∠P/dω = Re(Σ k p_k z⁻ᵏ / Σ p_k z⁻ᵏ) [samples].
    const auto delay = [&](double p0, double p1, double p2) {
        return std::real((p1 * z1 + 2.0 * p2 * z2) / (p0 + p1 * z1 + p2 * z2));
    };
    return (delay(d(c.b0), d(c.b1), d(c.b2)) - delay(1.0, d(c.a1), d(c.a2))) / fs;
}

/// A second-order IIR section in Direct Form I. See the file comment for the
/// model and invariants.
template <typename T>
class Biquad {
public:
    /// The pass-through section H(z) = 1 at rest.
    Biquad() noexcept = default;

    /// Pre:  biquad_coefficients_valid(c); `initial` finite.
    /// Post: the history is the steady state for a constant input `initial`
    ///       (x = initial, y = H(1)·initial), so a filter started on a live
    ///       signal has no start-up transient. Establishes I1, I2. In a
    ///       release build, invalid coefficients give the pass-through section
    ///       and a non-finite `initial` gives 0.
    explicit Biquad(const BiquadCoefficients<T>& c, T initial = T(0)) noexcept {
        if (biquad_coefficients_valid(c)) {
            c_ = c;
        } else {
            assert(false && "Biquad: coefficients must be finite and stable");  // Pre
        }
        reset(initial);
    }

    /// Filter one sample.
    ///
    /// Pre:  none. A non-finite input, or an output that overflows the scalar
    ///       type, is a sensor fault: the section returns its last output and
    ///       keeps its state, so one bad sample does not poison the filter.
    /// Post: returns y[n] and shifts the history. I1, I2 hold.
    T update(T x) noexcept {
        using std::isfinite;
        if (!isfinite(x)) {
            return y1_;  // fault: hold (I2 untouched)
        }
        const T y = c_.b0 * x + c_.b1 * x1_ + c_.b2 * x2_ - c_.a1 * y1_ - c_.a2 * y2_;
        if (!isfinite(y)) {
            return y1_;  // overflow: hold (I2 untouched)
        }
        x2_ = x1_;
        x1_ = x;
        y2_ = y1_;
        y1_ = y;
        assert(check_invariant());
        return y;
    }

    /// Replace the coefficients and keep the history, so a retuned section
    /// continues from the current signal without a jump.
    ///
    /// Pre:  biquad_coefficients_valid(c). Post: coefficients() == c. In a
    ///       release build, invalid coefficients are ignored (I1 is kept).
    void set_coefficients(const BiquadCoefficients<T>& c) noexcept {
        if (!biquad_coefficients_valid(c)) {
            assert(false && "Biquad: coefficients must be finite and stable");  // Pre
            return;
        }
        c_ = c;
        assert(check_invariant());
    }

    /// Restart at the steady state for a constant input `initial`.
    /// Pre: `initial` finite (otherwise 0 is used). Post: as the constructor.
    void reset(T initial = T(0)) noexcept {
        using std::isfinite;
        if (!isfinite(initial)) {
            assert(false && "Biquad: initial value must be finite");  // Pre
            initial = T(0);
        }
        x1_ = initial;
        x2_ = initial;
        y1_ = dc_gain(c_) * initial;
        y2_ = y1_;
        assert(check_invariant());
    }

    [[nodiscard]] const BiquadCoefficients<T>& coefficients() const noexcept {
        return c_;
    }

    /// The last output y[n−1] (the steady-state output before any update).
    [[nodiscard]] T output() const noexcept {
        return y1_;
    }

private:
    [[nodiscard]] bool check_invariant() const noexcept {
        using std::isfinite;
        return biquad_coefficients_valid(c_)                                         // I1
               && isfinite(x1_) && isfinite(x2_) && isfinite(y1_) && isfinite(y2_);  // I2
    }

    BiquadCoefficients<T> c_{};
    T x1_{0};
    T x2_{0};
    T y1_{0};
    T y2_{0};
};

}  // namespace branes::reflex::dsp

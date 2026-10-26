// Shared harness for the dsp/ tests: impulse-response norms, the rounding
// bound of a Direct Form I section, and steady-state tone measurement.
//
// Tolerances come from these bounds, never from observed results:
//  - Transient. A section started from zero and driven by a unit tone differs
//    from its steady state by at most the tail Σ_{k≥n} |h[k]| of its impulse
//    response, so measuring after n samples costs at most tail(n).
//  - Rounding. One Direct Form I step rounds five products and four sums, so
//    its error is at most γ5·(Σ|b|·X + (|a1| + |a2|)·Y), γ5 = 5ε/(1 − 5ε), for
//    inputs bounded by X and outputs by Y; the errors then pass through
//    1/A(z), whose ℓ1 norm bounds their sum. Rounding the input to the scalar
//    type adds ε·X·‖h‖₁.
#pragma once

#include <branes/reflex/dsp/biquad.hpp>

#include <cmath>
#include <complex>
#include <cstddef>
#include <limits>
#include <numbers>
#include <vector>

namespace dsp_harness {

using branes::reflex::dsp::Biquad;
using branes::reflex::dsp::BiquadCoefficients;

constexpr double kPi = std::numbers::pi;

template <typename T>
BiquadCoefficients<double> to_double(const BiquadCoefficients<T>& c) {
    return {static_cast<double>(c.b0),
            static_cast<double>(c.b1),
            static_cast<double>(c.b2),
            static_cast<double>(c.a1),
            static_cast<double>(c.a2)};
}

/// The impulse response h[0..n) of `c`, in double (no fault handling: the
/// raw recurrence).
inline std::vector<double> impulse_response(const BiquadCoefficients<double>& c, std::size_t n) {
    std::vector<double> h(n);
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double x = k == 0 ? 1.0 : 0.0;
        const double y = c.b0 * x + c.b1 * x1 + c.b2 * x2 - c.a1 * y1 - c.a2 * y2;
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
        h[k] = y;
    }
    return h;
}

/// Norms of a stable section's impulse response, computed out to where the
/// remaining tail is below 1e-17 of the total (the response decays
/// geometrically at the pole radius).
struct Norms {
    double l1;                 ///< ‖h‖₁
    std::vector<double> tail;  ///< tail[n] = Σ_{k≥n} |h[k]|
};

inline Norms norms(const BiquadCoefficients<double>& c) {
    std::size_t n = 1024;
    for (;;) {
        const std::vector<double> h = impulse_response(c, n);
        double last = 0;
        for (std::size_t k = n - n / 8; k < n; ++k) {
            last += std::abs(h[k]);
        }
        if (last < 1e-18 || n >= (std::size_t{1} << 24)) {
            Norms out{0, std::vector<double>(n + 1, 0.0)};
            for (std::size_t k = n; k-- > 0;) {
                out.tail[k] = out.tail[k + 1] + std::abs(h[k]);
            }
            out.l1 = out.tail[0];
            return out;
        }
        n *= 2;
    }
}

/// Unit roundoff of T.
template <typename T>
double eps() {
    return static_cast<double>(std::numeric_limits<T>::epsilon()) / 2;
}

/// Bound on the accumulated rounding error of a Biquad<T> with coefficients
/// `c` driven by inputs bounded by `x_max` (see the file comment).
template <typename T>
double rounding_bound(const BiquadCoefficients<T>& ct, double x_max) {
    const BiquadCoefficients<double> c = to_double(ct);
    const double u = eps<T>();
    const double gamma5 = 5 * u / (1 - 5 * u);
    const double h_l1 = norms(c).l1;
    const double y_max = h_l1 * x_max * (1 + 1e-3);  // margin for the rounding itself, ≪ 1 in every type tested
    const double step = gamma5 * ((std::abs(c.b0) + std::abs(c.b1) + std::abs(c.b2)) * x_max +
                                  (std::abs(c.a1) + std::abs(c.a2)) * y_max);
    const double feedback_l1 = norms(BiquadCoefficients<double>{1, 0, 0, c.a1, c.a2}).l1;
    return step * feedback_l1 + u * x_max * h_l1;
}

/// Settling samples after which the transient of a unit tone is below `bound`.
inline std::size_t settle_samples(const Norms& n, double bound) {
    std::size_t k = 0;
    while (k + 1 < n.tail.size() && n.tail[k] > bound) {
        ++k;
    }
    return k;
}

/// Steady-state complex gain of a Biquad<T>, measured by driving it with a
/// unit cosine at f and projecting `periods_samples` samples (a whole number
/// of periods) onto cos and sin after `settle` samples.
template <typename T>
std::complex<double>
measure_gain(const BiquadCoefficients<T>& c, double f, double fs, std::size_t settle, std::size_t window) {
    Biquad<T> b(c);
    double in_phase = 0;
    double quadrature = 0;
    for (std::size_t k = 0; k < settle + window; ++k) {
        const double theta = 2 * kPi * f * static_cast<double>(k) / fs;
        const double y = static_cast<double>(b.update(T(std::cos(theta))));
        if (k >= settle) {
            in_phase += y * std::cos(theta);
            quadrature += y * std::sin(theta);
        }
    }
    const double scale = 2.0 / static_cast<double>(window);
    // y = |H| cos(θ + φ) projects to (|H| cos φ, −|H| sin φ).
    return {in_phase * scale, -quadrature * scale};
}

}  // namespace dsp_harness

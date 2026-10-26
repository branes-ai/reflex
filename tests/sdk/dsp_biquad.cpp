// branes::reflex::dsp biquad sections: validators, contracts, the designs
// against their analytic properties, measured responses against response()
// and group_delay(), and a code-to-code comparison with stillwater-sc/mp-dsp
// (an independent implementation). Acceptance criteria: reflex#21.

#include <branes/reflex/dsp/biquad.hpp>

#include "dsp_harness.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <sw/dsp/filter/biquad/state.hpp>
#include <sw/dsp/filter/iir/butterworth.hpp>
#include <sw/dsp/filter/iir/rbj.hpp>
#include <universal/number/cfloat/cfloat.hpp>
#include <universal/number/posit/posit.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

using namespace branes::reflex::dsp;
using namespace dsp_harness;

namespace {

using posit16 = sw::universal::posit<16, 1>;
using binary16 = sw::universal::cfloat<16, 5, std::uint16_t, true, false, false>;  // IEEE-754 binary16 layout

constexpr double kFs = 8000.0;  // gyro rate [Hz]

/// Largest coefficient difference relative to max(1, |c|).
template <typename A, typename B>
double coefficient_distance(const BiquadCoefficients<A>& a, const BiquadCoefficients<B>& b) {
    const BiquadCoefficients<double> x = to_double(a);
    const BiquadCoefficients<double> y = to_double(b);
    const auto rel = [](double p, double q) { return std::abs(p - q) / std::max(1.0, std::abs(q)); };
    return std::max({rel(x.b0, y.b0), rel(x.b1, y.b1), rel(x.b2, y.b2), rel(x.a1, y.a1), rel(x.a2, y.a2)});
}

/// Frequencies on the measurement grid (whole periods in `window` samples).
std::vector<double> tone_grid(std::size_t window) {
    const double step = kFs / static_cast<double>(window);
    std::vector<double> f;
    for (int i = 0; i < 20; ++i) {
        const double target = 2.0 * std::pow(1800.0, i / 19.0);  // 2 Hz … 3600 Hz, log-spaced
        f.push_back(std::max(step, std::round(target / step) * step));
    }
    return f;
}

}  // namespace

// ── Validators and contracts ──────────────────────────────────────────────

TEST_CASE("biquad_coefficients_valid accepts the stability triangle's interior only", "[dsp][biquad]") {
    REQUIRE(biquad_coefficients_valid(BiquadCoefficients<double>{}));
    REQUIRE(biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, 0, -1.9, 0.95}));
    REQUIRE_FALSE(biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, 0, 0, 1.0}));  // |a2| = 1
    REQUIRE_FALSE(
        biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, 0, -1.5, 0.5}));  // |a1| = 1 + a2: pole at z = 1
    REQUIRE_FALSE(biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, 0, 1.5, 0.5}));  // pole at z = −1
    REQUIRE_FALSE(biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, 0, 0, -1.2}));
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    REQUIRE_FALSE(biquad_coefficients_valid(BiquadCoefficients<double>{nan, 0, 0, 0, 0}));
    REQUIRE_FALSE(biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, inf, 0, 0}));
    REQUIRE_FALSE(biquad_coefficients_valid(BiquadCoefficients<double>{1, 0, 0, nan, 0}));
}

TEST_CASE("design_frequency_valid requires 0 < f < fs/2", "[dsp][biquad]") {
    REQUIRE(design_frequency_valid(1.0, kFs));
    REQUIRE(design_frequency_valid(3999.0, kFs));
    REQUIRE_FALSE(design_frequency_valid(0.0, kFs));
    REQUIRE_FALSE(design_frequency_valid(4000.0, kFs));
    REQUIRE_FALSE(design_frequency_valid(-1.0, kFs));
    REQUIRE_FALSE(design_frequency_valid(100.0, std::numeric_limits<double>::infinity()));
    REQUIRE_FALSE(design_frequency_valid(std::numeric_limits<double>::quiet_NaN(), kFs));
}

TEMPLATE_TEST_CASE("Biquad starts at the steady state of its initial value", "[dsp][biquad]", double, float) {
    using T = TestType;
    Biquad<T> lp(butterworth_lowpass(T(150), T(kFs)), T(2));
    REQUIRE(lp.output() == T(2) * dc_gain(lp.coefficients()));
    // A constant input at the initial value produces no transient: every
    // output stays within the rounding bound of the steady state.
    const double bound = rounding_bound(lp.coefficients(), 2.0);
    for (int k = 0; k < 2000; ++k) {
        const double y = static_cast<double>(lp.update(T(2)));
        INFO("sample " << k);
        REQUIRE(std::abs(y - 2.0 * static_cast<double>(dc_gain(lp.coefficients()))) <= bound);
    }
}

TEMPLATE_TEST_CASE("Biquad holds its output and state on a non-finite input", "[dsp][biquad]", double, float) {
    using T = TestType;
    const BiquadCoefficients<T> c = butterworth_lowpass(T(150), T(kFs));
    Biquad<T> faulted(c);
    Biquad<T> clean(c);
    std::mt19937 rng(20261026);
    std::uniform_real_distribution<double> u(-1, 1);
    for (int k = 0; k < 500; ++k) {
        const T x = T(u(rng));
        const T y = faulted.update(x);
        REQUIRE(y == clean.update(x));
        if (k % 50 == 25) {
            // A fault returns the last output and leaves the history alone, so
            // the faulted section keeps tracking the clean one bit for bit.
            REQUIRE(faulted.update(std::numeric_limits<T>::quiet_NaN()) == y);
            REQUIRE(faulted.update(std::numeric_limits<T>::infinity()) == y);
            REQUIRE(faulted.update(-std::numeric_limits<T>::infinity()) == y);
            REQUIRE(faulted.output() == y);
        }
    }
}

TEMPLATE_TEST_CASE("Biquad holds on an output overflow", "[dsp][biquad]", double, float) {
    using T = TestType;
    // A gain-of-four section driven at the largest finite value overflows.
    Biquad<T> b(BiquadCoefficients<T>{T(4), T(0), T(0), T(0), T(0)});
    REQUIRE(b.update(T(1)) == T(4));
    REQUIRE(b.update(std::numeric_limits<T>::max()) == T(4));
    REQUIRE(b.update(T(1)) == T(4));  // the history did not take the overflow
}

TEMPLATE_TEST_CASE("set_coefficients keeps the history", "[dsp][biquad]", double, float) {
    using T = TestType;
    Biquad<T> b(notch(T(200), T(5), T(kFs)));
    T x1{0}, x2{0}, y1{0}, y2{0};
    for (int k = 0; k < 40; ++k) {
        const T x = T(std::sin(0.3 * k));
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = b.update(x);
    }
    const BiquadCoefficients<T> c = notch(T(300), T(5), T(kFs));
    b.set_coefficients(c);
    REQUIRE(b.coefficients().b1 == c.b1);
    REQUIRE(b.output() == y1);
    // The next output is the new section applied to the old history.
    const T x = T(0.25);
    const T expected = c.b0 * x + c.b1 * x1 + c.b2 * x2 - c.a1 * y1 - c.a2 * y2;
    REQUIRE(b.update(x) == expected);
}

TEMPLATE_TEST_CASE("reset restarts at a new steady state", "[dsp][biquad]", double, float) {
    using T = TestType;
    Biquad<T> b(first_order_lowpass(T(80), T(kFs)));
    for (int k = 0; k < 10; ++k) {
        (void)b.update(T(static_cast<double>(k)));
    }
    b.reset(T(-1));
    REQUIRE(b.output() == -dc_gain(b.coefficients()));
    b.reset();
    REQUIRE(b.output() == T(0));
}

// ── Designs against their analytic properties ─────────────────────────────

TEST_CASE("butterworth_lowpass: unity DC, −3 dB at fc, monotone magnitude", "[dsp][biquad][design]") {
    for (const double fc : {30.0, 150.0, 400.0, 1500.0, 3500.0}) {
        INFO("fc " << fc << " Hz");
        const BiquadCoefficients<double> c = butterworth_lowpass(fc, kFs);
        REQUIRE(biquad_coefficients_valid(c));
        REQUIRE(std::abs(std::abs(response(c, 0.0, kFs)) - 1.0) <= 1e-12);
        REQUIRE(std::abs(std::abs(response(c, fc, kFs)) - 1.0 / std::numbers::sqrt2) <= 1e-12);
        double previous = 2.0;
        for (int i = 0; i <= 1000; ++i) {
            const double m = std::abs(response(c, kFs / 2 * i / 1000.0, kFs));
            REQUIRE(m <= previous);
            previous = m;
        }
        REQUIRE(previous <= 1e-12);  // the double zero at z = −1
    }
}

TEST_CASE("first_order_lowpass: unity DC, −3 dB at fc", "[dsp][biquad][design]") {
    for (const double fc : {10.0, 80.0, 600.0}) {
        INFO("fc " << fc << " Hz");
        const BiquadCoefficients<double> c = first_order_lowpass(fc, kFs);
        REQUIRE(biquad_coefficients_valid(c));
        REQUIRE(c.b2 == 0.0);
        REQUIRE(c.a2 == 0.0);
        REQUIRE(std::abs(std::abs(response(c, 0.0, kFs)) - 1.0) <= 1e-12);
        REQUIRE(std::abs(std::abs(response(c, fc, kFs)) - 1.0 / std::numbers::sqrt2) <= 1e-12);
    }
}

TEST_CASE("notch: zero at f0, unity at DC and Nyquist, closed-form −3 dB width", "[dsp][biquad][design]") {
    for (const double f0 : {100.0, 264.0, 800.0, 3000.0}) {
        for (const double q : {2.0, 5.0, 20.0}) {
            INFO("f0 " << f0 << " Hz, q " << q);
            const BiquadCoefficients<double> c = notch(f0, q, kFs);
            REQUIRE(biquad_coefficients_valid(c));
            REQUIRE(std::abs(response(c, f0, kFs)) <= 1e-12);
            REQUIRE(std::abs(std::abs(response(c, 0.0, kFs)) - 1.0) <= 1e-12);
            REQUIRE(std::abs(std::abs(response(c, kFs / 2, kFs)) - 1.0) <= 1e-12);
            // |H|² = (cos ω − cos ω0)² / ((cos ω − cos ω0)² + α² sin² ω), so
            // the −3 dB edges solve cos ω ∓ α sin ω = cos ω0:
            // ω = acos(cos ω0 / √(1 + α²)) ∓ atan α.
            const double w0 = 2 * kPi * f0 / kFs;
            const double alpha = std::sin(w0) / (2 * q);
            const double mid = std::acos(std::cos(w0) / std::sqrt(1 + alpha * alpha));
            for (const double w : {mid - std::atan(alpha), mid + std::atan(alpha)}) {
                REQUIRE(std::abs(std::abs(response(c, w * kFs / (2 * kPi), kFs)) - 1.0 / std::numbers::sqrt2) <= 1e-12);
            }
            const double width_hz = kFs / kPi * std::atan(alpha);
            std::printf("notch f0 %6.0f Hz q %4.1f: -3 dB width %8.3f Hz, f0/q %8.3f Hz\n", f0, q, width_hz, f0 / q);
        }
    }
}

TEMPLATE_TEST_CASE("designs in T stay within their rounding of the double design", "[dsp][biquad][design]", float) {
    using T = TestType;
    // Each coefficient takes at most 16 roundings (sin, cos, and ≤ 14 + − × ÷
    // with no cancellation in these formulas), relative to max(1, |c|).
    const double bound = 16 * eps<T>();
    for (const double f : {30.0, 150.0, 264.0, 1500.0}) {
        INFO("f " << f << " Hz");
        REQUIRE(coefficient_distance(butterworth_lowpass(T(f), T(kFs)), butterworth_lowpass(f, kFs)) <= bound);
        REQUIRE(coefficient_distance(first_order_lowpass(T(f), T(kFs)), first_order_lowpass(f, kFs)) <= bound);
        REQUIRE(coefficient_distance(notch(T(f), T(5), T(kFs)), notch(f, 5.0, kFs)) <= bound);
    }
}

// ── Measured responses against the analytic ones ──────────────────────────

TEMPLATE_TEST_CASE("measured tone gains match response() at 20 frequencies",
                   "[dsp][biquad][verification]",
                   double,
                   float) {
    using T = TestType;
    constexpr std::size_t kWindow = 4000;  // 2 Hz grid at 8 kHz
    const std::vector<double> grid = tone_grid(kWindow);
    const BiquadCoefficients<T> designs[] = {
        butterworth_lowpass(T(150), T(kFs)), first_order_lowpass(T(80), T(kFs)), notch(T(264), T(5), T(kFs))};
    for (const BiquadCoefficients<T>& c : designs) {
        const Norms n = norms(to_double(c));
        const std::size_t settle = settle_samples(n, 1e-14);
        // Measured gain error: twice the per-sample error (the projection's
        // scale), plus the projection's own double rounding.
        const double tol = 2 * (n.tail[settle] + rounding_bound(c, 1.0)) + 2 * kWindow * eps<double>() * n.l1;
        for (const double f : grid) {
            const std::complex<double> measured = measure_gain(c, f, kFs, settle, kWindow);
            const std::complex<double> analytic = response(c, f, kFs);
            INFO("b1 " << static_cast<double>(c.b1) << ", f " << f << " Hz: measured " << measured << ", analytic "
                       << analytic << ", tolerance " << tol);
            REQUIRE(std::abs(measured - analytic) <= tol);
        }
    }
}

TEMPLATE_TEST_CASE("a tone at the notch centre is attenuated >= 60 dB (double), >= 40 dB (float)",
                   "[dsp][biquad][verification]",
                   double,
                   float) {
    using T = TestType;
    const double floor_db = std::is_same_v<T, double> ? -60.0 : -40.0;
    for (const double f0 : {100.0, 264.0, 800.0, 3000.0}) {
        const BiquadCoefficients<T> c = notch(T(f0), T(5), T(kFs));
        const std::size_t settle = settle_samples(norms(to_double(c)), 1e-14);
        const double gain_db = 20 * std::log10(std::abs(measure_gain(c, f0, kFs, settle, 4000)));
        INFO("f0 " << f0 << " Hz: " << gain_db << " dB");
        REQUIRE(gain_db <= floor_db);
    }
}

TEMPLATE_TEST_CASE("measured group delay matches group_delay() to 50 us",
                   "[dsp][biquad][verification]",
                   double,
                   float) {
    using T = TestType;
    constexpr std::size_t kWindow = 4000;
    const double df = kFs / kWindow;
    const BiquadCoefficients<T> designs[] = {butterworth_lowpass(T(150), T(kFs)), first_order_lowpass(T(80), T(kFs))};
    for (const BiquadCoefficients<T>& c : designs) {
        const std::size_t settle = settle_samples(norms(to_double(c)), 1e-14);
        for (const double f : {10.0, 50.0, 100.0}) {
            // −dφ/dω from the measured phases one grid step either side.
            const double phase_hi = std::arg(measure_gain(c, f + df, kFs, settle, kWindow));
            const double phase_lo = std::arg(measure_gain(c, f - df, kFs, settle, kWindow));
            const double measured = -(phase_hi - phase_lo) / (2 * kPi * 2 * df);
            const double analytic = group_delay(c, f, kFs);
            INFO("f " << f << " Hz: measured " << measured * 1e3 << " ms, closed form " << analytic * 1e3 << " ms");
            REQUIRE(std::abs(measured - analytic) <= 50e-6);
        }
    }
}

TEST_CASE("group_delay's closed form matches the derivative of response()'s phase", "[dsp][biquad][verification]") {
    // Central difference with step h has error ≤ h²/6·|φ'''|; with h = 1e-3 Hz
    // that is far below 1e-9 s for these sections, and the phase is evaluated
    // to ~1e-16 rad, so 1e-9 s bounds both.
    for (const BiquadCoefficients<double>& c :
         {butterworth_lowpass(150.0, kFs), first_order_lowpass(80.0, kFs), notch(264.0, 5.0, kFs)}) {
        for (const double f : {5.0, 40.0, 150.0, 400.0, 1200.0}) {
            const double h = 1e-3;
            const double dphi = std::arg(response(c, f + h, kFs) / response(c, f - h, kFs));
            const double numeric = -dphi / (2 * kPi * 2 * h);
            INFO("f " << f << " Hz");
            REQUIRE(std::abs(numeric - group_delay(c, f, kFs)) <= 1e-9);
        }
    }
}

// ── Code-to-code: stillwater-sc/mp-dsp (BB-V6) ────────────────────────────

TEMPLATE_TEST_CASE("designs match mp-dsp's independent implementation", "[dsp][biquad][reference]", double, float) {
    using T = TestType;
    // mp-dsp designs the Butterworth from its analog prototype's poles and the
    // notch from the cookbook in its own code; both reach the same rational
    // coefficients, so each coefficient may differ by the two sides' rounding.
    const double bound = 4 * eps<T>();
    for (const double fc : {30.0, 150.0, 400.0, 1500.0}) {
        sw::dsp::iir::ButterworthLowPass<2, T> theirs;
        theirs.setup(2, kFs, fc);
        const auto& s = theirs.cascade().stage(0);
        const BiquadCoefficients<T> ref{s.b0, s.b1, s.b2, s.a1, s.a2};
        INFO("butterworth fc " << fc << " Hz: distance "
                               << coefficient_distance(butterworth_lowpass(T(fc), T(kFs)), ref) << ", bound " << bound);
        REQUIRE(coefficient_distance(butterworth_lowpass(T(fc), T(kFs)), ref) <= bound);
    }
    for (const double f0 : {100.0, 264.0, 800.0, 3000.0}) {
        sw::dsp::iir::rbj::BandStop<T> theirs;
        theirs.setup(kFs, f0, 5.0);  // the third argument is q (stillwater-sc/mp-dsp#218)
        const auto& s = theirs.cascade().stage(0);
        const BiquadCoefficients<T> ref{s.b0, s.b1, s.b2, s.a1, s.a2};
        INFO("notch f0 " << f0 << " Hz: distance " << coefficient_distance(notch(T(f0), T(5), T(kFs)), ref)
                         << ", bound " << bound);
        REQUIRE(coefficient_distance(notch(T(f0), T(5), T(kFs)), ref) <= bound);
    }
}

TEMPLATE_TEST_CASE("filter outputs match mp-dsp's Direct Form I sample by sample",
                   "[dsp][biquad][reference]",
                   double,
                   float) {
    using T = TestType;
    std::mt19937 rng(20261026);
    std::uniform_real_distribution<double> u(-1, 1);
    std::vector<double> input(20000);
    for (double& x : input) {
        x = u(rng);
    }
    for (const double f : {150.0, 264.0, 1500.0}) {
        for (int kind = 0; kind < 2; ++kind) {
            BiquadCoefficients<T> ours = kind == 0 ? butterworth_lowpass(T(f), T(kFs)) : notch(T(f), T(5), T(kFs));
            sw::dsp::BiquadCoefficients<T> theirs;
            if (kind == 0) {
                sw::dsp::iir::ButterworthLowPass<2, T> d;
                d.setup(2, kFs, f);
                theirs = d.cascade().stage(0);
            } else {
                sw::dsp::iir::rbj::BandStop<T> d;
                d.setup(kFs, f, 5.0);
                theirs = d.cascade().stage(0);
            }
            // Each side's rounding, plus the effect of the coefficient
            // difference: coefficient i differs by at most d·max(1, |c_i|)
            // (d the measured distance), which changes each step by that times
            // |x| ≤ 1 (numerator) or |y| ≤ ‖h‖₁ (denominator), through 1/A.
            const BiquadCoefficients<T> ref{theirs.b0, theirs.b1, theirs.b2, theirs.a1, theirs.a2};
            const BiquadCoefficients<double> cd = to_double(ours);
            const double h_l1 = norms(cd).l1;
            const double feedback_l1 = norms(BiquadCoefficients<double>{1, 0, 0, cd.a1, cd.a2}).l1;
            const double d = coefficient_distance(ours, ref);
            const auto m = [](double v) { return std::max(1.0, std::abs(v)); };
            const double step = d * ((m(cd.b0) + m(cd.b1) + m(cd.b2)) + (m(cd.a1) + m(cd.a2)) * h_l1);
            const double tol = rounding_bound(ours, 1.0) + rounding_bound(ref, 1.0) + step * feedback_l1;
            Biquad<T> mine(ours);
            sw::dsp::DirectFormI<T> state;
            double worst = 0;
            for (const double x : input) {
                const double a = static_cast<double>(mine.update(T(x)));
                const double b = static_cast<double>(state.process(T(x), theirs));
                worst = std::max(worst, std::abs(a - b));
            }
            INFO((kind == 0 ? "butterworth " : "notch ")
                 << f << " Hz: worst difference " << worst << ", bound " << tol);
            REQUIRE(worst <= tol);
        }
    }
}

// ── Characterization, not gated: 16-bit types (BB-S4) ─────────────────────

TEST_CASE("16-bit types: notch depth and low-pass error, reported", "[dsp][biquad][characterization]") {
    const auto report = [](const char* name, auto zero) {
        using T = decltype(zero);
        for (const double f0 : {100.0, 264.0, 800.0}) {
            const BiquadCoefficients<T> c = notch(T(f0), T(5), T(kFs));
            const bool valid = biquad_coefficients_valid(c);
            double depth = 0;
            if (valid) {
                Biquad<T> b(c);
                double in = 0, out = 0;
                for (int k = 0; k < 16000; ++k) {
                    const double x = std::sin(2 * kPi * f0 * k / kFs);
                    const double y = static_cast<double>(b.update(T(x)));
                    if (k >= 8000) {
                        in += x * x;
                        out += y * y;
                    }
                }
                depth = 10 * std::log10(out / in);
            }
            std::printf("%-12s notch f0 %4.0f Hz q 5: %s%6.1f dB at f0\n", name, f0, valid ? "" : "INVALID ", depth);
        }
    };
    report("double", 0.0);
    report("float", 0.0f);
    report("posit<16,1>", posit16(0));
    report("binary16", binary16(0));
    SUCCEED();
}

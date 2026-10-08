// Gyro low-pass filter in mixed precision — an end-to-end example that runs a
// DSP filter through MTL5 and Universal, so CI exercises both dependencies.
//
// Flight controllers low-pass the gyro to strip motor/propeller vibration
// before the rate PID sees it. This example builds that filter: a 2nd-order
// Butterworth low-pass (fc = 30 Hz, fs = 1 kHz), designed with the bilinear
// transform and realized in state-space form with MTL5 dense matrices
//
//   x[k+1] = A x[k] + B u[k]          (transposed direct form II)
//   y[k]   = C x[k] + D u[k]
//
// It filters a synthetic gyro signal (2 Hz body motion + 180 Hz motor
// vibration) in double (the reference), float, posit<32,2>, and three 16-bit
// formats an embedded filter might use: IEEE binary16 (cfloat<16,5>), posit<16,1>,
// and Q3.12 fixed point. The design is checked against its specification
// (passband gain, stopband attenuation) and each arithmetic against the
// double reference.

#include <catch2/catch_test_macros.hpp>
#include <mtl/mtl.hpp>
#include <universal/number/cfloat/cfloat.hpp>
#include <universal/number/fixpnt/fixpnt.hpp>
#include <universal/number/posit/posit.hpp>

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <string>
#include <vector>

namespace {

using posit32 = sw::universal::posit<32, 2>;
using posit16 = sw::universal::posit<16, 1>;
using binary16 = sw::universal::cfloat<16, 5, std::uint16_t, true, false, false>;  // IEEE-754 binary16 layout
using q3_12 = sw::universal::fixpnt<16, 12, sw::universal::Saturate, std::uint16_t>;

constexpr double kFs = 1000.0;          // Hz, gyro sample rate
constexpr double kCutoff = 30.0;        // Hz
constexpr double kMotionHz = 2.0;       // body motion
constexpr double kMotionAmp = 0.5;      // rad/s
constexpr double kVibrationHz = 180.0;  // motor vibration
constexpr double kVibrationAmp = 0.3;   // rad/s
constexpr std::size_t kSamples = 2000;  // 2 s
constexpr std::size_t kSettle = 500;    // skip the first 0.5 s (start-up transient)

// 2nd-order Butterworth low-pass via the bilinear transform with
// frequency prewarping: H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2).
struct Biquad {
    double b0, b1, b2, a1, a2;
};

Biquad butterworth_lowpass(double fc, double fs) {
    const double k = std::tan(std::numbers::pi * fc / fs);
    const double q = std::numbers::sqrt2;
    const double norm = 1.0 / (1.0 + q * k + k * k);
    const double b0 = k * k * norm;
    return {b0, 2.0 * b0, b0, 2.0 * (k * k - 1.0) * norm, (1.0 - q * k + k * k) * norm};
}

// The biquad as a 2-state state-space filter, in arithmetic T.
template <typename T>
class StateSpaceFilter {
public:
    explicit StateSpaceFilter(const Biquad& f) : A_(2, 2), B_(2, 1), C_(1, 2), D_(1, 1), x_(2) {
        // Transposed direct form II: y = b0 u + x1,
        //   x1' = -a1 x1 + x2 + (b1 - a1 b0) u,   x2' = -a2 x1 + (b2 - a2 b0) u
        A_(0, 0) = T(-f.a1);
        A_(0, 1) = T(1);
        A_(1, 0) = T(-f.a2);
        A_(1, 1) = T(0);
        B_(0, 0) = T(f.b1 - f.a1 * f.b0);
        B_(1, 0) = T(f.b2 - f.a2 * f.b0);
        C_(0, 0) = T(1);
        C_(0, 1) = T(0);
        D_(0, 0) = T(f.b0);
        x_[0] = T(0);
        x_[1] = T(0);
    }

    T step(T input) {
        mtl::vec::dense_vector<T> u(1);
        u[0] = input;
        mtl::vec::dense_vector<T> y(1);
        y = C_ * x_ + D_ * u;
        mtl::vec::dense_vector<T> x_next(2);
        x_next = A_ * x_ + B_ * u;  // into a temporary: expressions must not alias
        x_ = x_next;
        return y[0];
    }

private:
    mtl::mat::dense2D<T> A_, B_, C_, D_;
    mtl::vec::dense_vector<T> x_;
};

double motion(std::size_t k) {
    return kMotionAmp * std::sin(2.0 * std::numbers::pi * kMotionHz * static_cast<double>(k) / kFs);
}

double vibration(std::size_t k) {
    return kVibrationAmp * std::sin(2.0 * std::numbers::pi * kVibrationHz * static_cast<double>(k) / kFs);
}

template <typename T>
std::vector<double> filter_signal(const Biquad& f, bool with_vibration) {
    StateSpaceFilter<T> filter{f};
    std::vector<double> out;
    out.reserve(kSamples);
    for (std::size_t k = 0; k < kSamples; ++k) {
        const double u = motion(k) + (with_vibration ? vibration(k) : 0.0);
        out.push_back(static_cast<double>(filter.step(T(u))));
    }
    return out;
}

// RMS of a - b over the settled part of the run (b may be empty: RMS of a).
double rms(const std::vector<double>& a, const std::vector<double>& b = {}) {
    double sum = 0.0;
    for (std::size_t k = kSettle; k < kSamples; ++k) {
        const double e = a[k] - (b.empty() ? 0.0 : b[k]);
        sum += e * e;
    }
    return std::sqrt(sum / static_cast<double>(kSamples - kSettle));
}

double db(double ratio) {
    return 20.0 * std::log10(ratio);
}

struct ArithmeticRun {
    const char* name;
    double error_rms;  // vs the double reference
    double snr_db;     // reference signal vs that error
    bool finite;
};

template <typename T>
ArithmeticRun run(const char* name, const Biquad& f, const std::vector<double>& reference) {
    const std::vector<double> out = filter_signal<T>(f, true);
    bool finite = true;
    for (double v : out) {
        finite = finite && std::isfinite(v);
    }
    const double err = rms(out, reference);
    return {name, err, err > 0.0 ? db(rms(reference) / err) : 999.0, finite};
}

// Frequency response H(e^{j 2 pi f / fs}) of the biquad, evaluated directly
// from its transfer function.
std::complex<double> response(const Biquad& f, double hz) {
    const std::complex<double> z_inv = std::polar(1.0, -2.0 * std::numbers::pi * hz / kFs);
    return (f.b0 + f.b1 * z_inv + f.b2 * z_inv * z_inv) / (1.0 + f.a1 * z_inv + f.a2 * z_inv * z_inv);
}

// Phase delay [s]: how far a sinusoid at `hz` is shifted in time.
double phase_delay_s(const Biquad& f, double hz) {
    return -std::arg(response(f, hz)) / (2.0 * std::numbers::pi * hz);
}

// Group delay [s]: -d(phase)/d(omega), by central difference (the phase is
// smooth and far from +/-pi wrap at the frequencies used here).
double group_delay_s(const Biquad& f, double hz) {
    const double dh = 1e-3;
    const double dphi = std::arg(response(f, hz + dh) / response(f, hz - dh));
    return -dphi / (2.0 * std::numbers::pi * 2.0 * dh);
}

// Phase [rad] of a sampled sinusoid at kMotionHz, by projecting the settled
// samples onto sin and cos over a whole number of periods (kSettle..kSamples
// is exactly three 2 Hz periods at 1 kHz).
double motion_phase(const std::vector<double>& x) {
    double in_phase = 0.0;
    double quadrature = 0.0;
    for (std::size_t k = kSettle; k < kSamples; ++k) {
        const double w = 2.0 * std::numbers::pi * kMotionHz * static_cast<double>(k) / kFs;
        in_phase += x[k] * std::sin(w);
        quadrature += x[k] * std::cos(w);
    }
    return std::atan2(quadrature, in_phase);
}

// Opt-in trace export for the docs figures (docs-site/scripts/gen-gyro-figures.mjs):
// set REFLEX_DSP_TRACE_DIR to a directory and this writes gyro_lowpass.json.
void write_trace(const Biquad& f, const std::vector<double>& output, double measured_lag_s) {
    const char* dir = std::getenv("REFLEX_DSP_TRACE_DIR");
    if (dir == nullptr || *dir == '\0') {
        return;
    }
    const std::string path = std::string(dir) + "/gyro_lowpass.json";
    std::ofstream out(path);
    if (!out) {
        std::fprintf(stderr, "cannot write %s\n", path.c_str());
        return;
    }
    out.precision(9);
    out << "{\n  \"fs_hz\": " << kFs << ", \"fc_hz\": " << kCutoff << ",\n";
    out << "  \"motion_hz\": " << kMotionHz << ", \"motion_amp\": " << kMotionAmp << ",\n";
    out << "  \"vibration_hz\": " << kVibrationHz << ", \"vibration_amp\": " << kVibrationAmp << ",\n";
    out << "  \"settle_samples\": " << kSettle << ",\n";
    out << "  \"coefficients\": {\"b0\": " << f.b0 << ", \"b1\": " << f.b1 << ", \"b2\": " << f.b2
        << ", \"a1\": " << f.a1 << ", \"a2\": " << f.a2 << "},\n";
    out << "  \"measured_motion_lag_ms\": " << measured_lag_s * 1e3 << ",\n";
    out << "  \"response\": [";
    const double freqs[] = {kMotionHz, kCutoff, kVibrationHz};
    for (std::size_t i = 0; i < 3; ++i) {
        const auto h = response(f, freqs[i]);
        out << (i ? ", " : "") << "{\"hz\": " << freqs[i] << ", \"gain\": " << std::abs(h)
            << ", \"gain_db\": " << db(std::abs(h)) << ", \"phase_deg\": " << std::arg(h) * 180.0 / std::numbers::pi
            << ", \"phase_delay_ms\": " << phase_delay_s(f, freqs[i]) * 1e3
            << ", \"group_delay_ms\": " << group_delay_s(f, freqs[i]) * 1e3 << "}";
    }
    out << "],\n";
    auto series = [&](const char* name, auto&& value, bool last) {
        out << "  \"" << name << "\": [";
        for (std::size_t k = 0; k < kSamples; ++k) {
            out << (k ? "," : "") << value(k);
        }
        out << "]" << (last ? "\n" : ",\n");
    };
    out.precision(6);
    series("motion", [](std::size_t k) { return motion(k); }, false);
    series("vibration", [](std::size_t k) { return vibration(k); }, false);
    series("input", [](std::size_t k) { return motion(k) + vibration(k); }, false);
    series("output", [&](std::size_t k) { return output[k]; }, true);
    out << "}\n";
    std::printf("wrote %s\n", path.c_str());
}

}  // namespace

TEST_CASE("dsp: butterworth low-pass meets its spec in double", "[dsp][example]") {
    const Biquad f = butterworth_lowpass(kCutoff, kFs);
    const std::vector<double> motion_only = filter_signal<double>(f, false);
    const std::vector<double> with_vibration = filter_signal<double>(f, true);

    // Unity DC gain: b0 + b1 + b2 == 1 + a1 + a2.
    REQUIRE(std::abs((f.b0 + f.b1 + f.b2) - (1.0 + f.a1 + f.a2)) < 1e-12);

    // Passband: the 2 Hz motion comes through essentially untouched in
    // amplitude (|H(2 Hz)| ~ 1; the phase lag is the filter's price).
    const double motion_rms_in = kMotionAmp / std::numbers::sqrt2;
    const double passband_gain = rms(motion_only) / motion_rms_in;
    // Stopband: what the vibration contributes after filtering, relative to
    // what it contributed before.
    const double vibration_rms_in = kVibrationAmp / std::numbers::sqrt2;
    const double stopband_db = db(rms(with_vibration, motion_only) / vibration_rms_in);

    std::printf("Butterworth LPF fc=%.0f Hz fs=%.0f Hz: |H(%.0f Hz)| = %.5f, vibration at %.0f Hz %.1f dB\n",
                kCutoff,
                kFs,
                kMotionHz,
                passband_gain,
                kVibrationHz,
                stopband_db);

    REQUIRE(std::abs(passband_gain - 1.0) < 0.01);
    // An analog 2nd-order Butterworth gives -31 dB at 6x the cutoff; the
    // bilinear transform's warping only adds attenuation up there.
    REQUIRE(stopband_db < -30.0);
}

TEST_CASE("dsp: the same filter in 32-bit and 16-bit arithmetic", "[dsp][example][posit]") {
    const Biquad f = butterworth_lowpass(kCutoff, kFs);
    const std::vector<double> reference = filter_signal<double>(f, true);

    const ArithmeticRun runs[] = {
        run<float>("float", f, reference),
        run<posit32>("posit<32,2>", f, reference),
        run<binary16>("half (cfloat<16,5>)", f, reference),
        run<posit16>("posit<16,1>", f, reference),
        run<q3_12>("Q3.12 fixpnt<16,12>", f, reference),
    };

    std::printf("Gyro LPF error vs double reference (signal RMS %.4f rad/s):\n", rms(reference));
    for (const ArithmeticRun& r : runs) {
        std::printf("  %-22s error RMS %.3e rad/s   SNR %6.1f dB%s\n",
                    r.name,
                    r.error_rms,
                    r.snr_db,
                    r.finite ? "" : "   NON-FINITE");
    }

    for (const ArithmeticRun& r : runs) {
        INFO(r.name);
        REQUIRE(r.finite);
    }
    // 32-bit formats: indistinguishable from double for a gyro filter.
    REQUIRE(runs[0].snr_db > 100.0);
    REQUIRE(runs[1].snr_db > 100.0);
    // 16-bit formats: regression floors a few dB under what each achieves
    // (half ~37 dB, posit<16,1> ~50 dB, Q3.12 ~41 dB at v5.1.0).
    REQUIRE(runs[2].snr_db > 33.0);
    REQUIRE(runs[3].snr_db > 45.0);
    REQUIRE(runs[4].snr_db > 36.0);
}

TEST_CASE("dsp: the filter delays the motion by its phase delay", "[dsp][example]") {
    const Biquad f = butterworth_lowpass(kCutoff, kFs);

    // Analytic: the 2 Hz motion is shifted by the phase delay at 2 Hz, and in
    // the passband phase delay ~ group delay (a near-pure time shift).
    const double analytic_s = phase_delay_s(f, kMotionHz);
    const double group_s = group_delay_s(f, kMotionHz);

    // Measured: the phase of the filtered motion vs. the clean motion, from
    // the settled samples. Independent of the H(z) formula above.
    std::vector<double> clean(kSamples);
    for (std::size_t k = 0; k < kSamples; ++k) {
        clean[k] = motion(k);
    }
    const std::vector<double> motion_only = filter_signal<double>(f, false);
    const double dphi = motion_phase(clean) - motion_phase(motion_only);
    const double measured_s = dphi / (2.0 * std::numbers::pi * kMotionHz);

    std::printf("2 Hz motion delay: measured %.3f ms, phase delay %.3f ms, group delay %.3f ms (%.2f deg)\n",
                measured_s * 1e3,
                analytic_s * 1e3,
                group_s * 1e3,
                std::arg(response(f, kMotionHz)) * 180.0 / std::numbers::pi);

    REQUIRE(std::abs(measured_s - analytic_s) < 0.05e-3);  // agree to 50 us
    REQUIRE(analytic_s > 7.0e-3);                          // ~7.5 ms at fc = 30 Hz
    REQUIRE(analytic_s < 8.0e-3);
    REQUIRE(std::abs(group_s - analytic_s) < 0.1e-3);  // passband: ~ pure time shift

    write_trace(f, filter_signal<double>(f, true), measured_s);
}

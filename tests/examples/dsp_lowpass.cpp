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
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numbers>
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

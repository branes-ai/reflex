#pragma once

// Shared by the gyro filter trace scenarios: a synthetic gyro signal (body
// motion plus vibration at the rotor harmonics, plus sensor noise), a rotor
// speed sweep, and a short-time amplitude spectrum for the spectrogram
// panels. Not a scenario itself (CMake builds only the .cpp files in this
// directory).
//
// The vibration is a sum of sines, one per rotor and harmonic, each with its
// phase integrated from the rotor speed, so a moving rotor sweeps its tones
// without phase jumps. Harmonic h of a rotor at speed ω has amplitude
//   a_h · (ω/ω_hover)²,  a = (0.1, 0.05, 0.03) rad/s,
// since an imbalance force grows with the square of the speed. These
// amplitudes are illustrative, not RP-1 flight data (the control guide's F3
// asks for the logs that would set them).
//
// The spectrum is computed here, in C++, so the player only paints recorded
// samples (ANI-1). It is the magnitude of a direct DFT of the last N samples
// under a Hann window, scaled so that a unit-amplitude sine at a bin centre
// reads 1. It is recorded as linear amplitude: the player converts to dB for
// the colour, which keeps the regen and wasm comparisons absolute.

#include <branes/reflex/dsp/rpm_notch.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include "../quadrotor/quadrotor_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <random>
#include <vector>

namespace branes::reflex::trace::filters {

namespace model = branes::reflex::model;
namespace dsp = branes::reflex::dsp;

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kGyroRate = 8000;  // gyro sample rate [Hz]
inline constexpr std::size_t kHarmonics = 3;
inline constexpr std::array<double, kHarmonics> kVibration = {0.1, 0.05, 0.03};  // rad/s at hover
inline constexpr double kNoise = 0.002;                                          // sensor noise RMS [rad/s]
/// Each rotor's speed relative to the fleet's: ±5 %, as in the RPM tracking test.
inline constexpr std::array<double, 4> kSpread = {0.95, 1 - 0.05 / 3, 1 + 0.05 / 3, 1.05};

/// RP-1's hover rotor speed [rad/s].
inline double hover_speed() {
    return model::QuadrotorModel<double>(model::QuadrotorParams<double>{}).hover_rotor_speed();
}

/// RP-1's maximum rotor speed [rad/s].
inline double max_speed() {
    return model::QuadrotorModel<double>(model::QuadrotorParams<double>{}).max_rotor_speed();
}

/// The four rotor speeds for a fleet speed w [rad/s].
inline std::array<double, 4> rotor_speeds(double w) {
    return {w * kSpread[0], w * kSpread[1], w * kSpread[2], w * kSpread[3]};
}

/// Rotor vibration and sensor noise, sampled at fs.
class Vibration {
public:
    explicit Vibration(double fs, std::uint32_t seed = 20261030) : fs_{fs}, rng_{seed}, hover_{hover_speed()} {}

    /// The next sample of the vibration, with the rotors at `omega` [rad/s]
    /// over this sample.
    double next(const std::array<double, 4>& omega) {
        double v = 0;
        for (std::size_t r = 0; r < 4; ++r) {
            const double s = omega[r] / hover_;
            for (std::size_t h = 0; h < kHarmonics; ++h) {
                v += kVibration[h] * s * s * std::sin(phase_[r][h]);
                phase_[r][h] = std::fmod(phase_[r][h] + static_cast<double>(h + 1) * omega[r] / fs_, 2 * kPi);
            }
        }
        return v;
    }

    /// The next sample of sensor noise: uniform, of RMS kNoise. It is kept
    /// apart from the vibration because no notch removes it: a scenario adds
    /// it to the motion, so the vibration a filter leaves is measured alone. The integer-to-double step is written out:
    /// The integer-to-double step is written out: std::uniform_real_distribution
    /// differs between standard libraries, and the wasm build (libc++) must
    /// match the native one (libstdc++).
    double noise() {
        const double u = (static_cast<double>(rng_()) + 0.5) / 4294967296.0;  // (0, 1)
        return kNoise * std::sqrt(12.0) * (u - 0.5);
    }

private:
    double fs_;
    std::mt19937 rng_;
    double hover_;
    std::array<std::array<double, kHarmonics>, 4> phase_{};
};

/// Short-time amplitude spectrum: bins 0 … bins−1 of an N-point DFT of the
/// last N samples, Hann windowed (see the file comment).
class Spectrum {
public:
    /// Pre: window ≥ 2, 1 ≤ bins ≤ window/2 + 1.
    Spectrum(double fs, std::size_t window, std::size_t bins)
        : fs_{fs}, n_{window}, bins_{bins}, buffer_(window, 0.0), cos_(window * bins), sin_(window * bins),
          hann_(window) {
        double sum = 0;
        for (std::size_t i = 0; i < n_; ++i) {
            hann_[i] = 0.5 - 0.5 * std::cos(2 * kPi * static_cast<double>(i) / static_cast<double>(n_));
            sum += hann_[i];
        }
        scale_ = 2 / sum;
        for (std::size_t k = 0; k < bins_; ++k) {
            for (std::size_t i = 0; i < n_; ++i) {
                // (k·i) mod N keeps the argument small, so cos and sin are exact to an ulp.
                const double a = 2 * kPi * static_cast<double>((k * i) % n_) / static_cast<double>(n_);
                cos_[k * n_ + i] = std::cos(a);
                sin_[k * n_ + i] = std::sin(a);
            }
        }
    }

    void push(double x) {
        buffer_[next_] = x;
        next_ = (next_ + 1) % n_;
    }

    /// Bin spacing [Hz].
    [[nodiscard]] double bin_hz() const {
        return fs_ / static_cast<double>(n_);
    }

    /// The amplitude per bin of the last N samples (zeros before the first).
    [[nodiscard]] std::vector<double> amplitudes() const {
        std::vector<double> out(bins_);
        for (std::size_t k = 0; k < bins_; ++k) {
            double re = 0;
            double im = 0;
            for (std::size_t i = 0; i < n_; ++i) {
                const double x = hann_[i] * buffer_[(next_ + i) % n_];  // oldest first
                re += x * cos_[k * n_ + i];
                im -= x * sin_[k * n_ + i];
            }
            out[k] = (k == 0 ? scale_ / 2 : scale_) * std::hypot(re, im);
        }
        return out;
    }

private:
    double fs_;
    std::size_t n_;
    std::size_t bins_;
    std::vector<double> buffer_;
    std::vector<double> cos_;
    std::vector<double> sin_;
    std::vector<double> hann_;
    double scale_ = 1;
    std::size_t next_ = 0;
};

/// Running sums of squares over a stretch of samples, for RMS ratios.
struct Energy {
    double sum = 0;
    void add(double x) {
        sum += x * x;
    }
};

inline constexpr double kFloorDb = -100;

/// 20·log10 of the RMS ratio out/in, no lower than kFloorDb; 0 dB when the
/// input had no energy. A settled notch leaves only roundoff (near −300 dB),
/// whose value in dB differs between compilers: the floor keeps the regen and
/// wasm comparisons meaningful.
inline double ratio_db(const Energy& out, const Energy& in) {
    if (!(in.sum > 0)) {
        return 0.0;
    }
    return std::max(kFloorDb, 10 * std::log10(out.sum / in.sum));
}

/// Every notch centre and weight of a 4-rotor RpmNotch, rotor-major.
template <std::size_t MaxHarmonics>
inline void notch_state(const dsp::RpmNotch<double, 4, MaxHarmonics>& n,
                        std::vector<double>& centers,
                        std::vector<double>& weights) {
    centers.clear();
    weights.clear();
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t h = 0; h < MaxHarmonics; ++h) {
            centers.push_back(n.center_hz(r, h));
            weights.push_back(n.weight(r, h));
        }
    }
}

}  // namespace branes::reflex::trace::filters

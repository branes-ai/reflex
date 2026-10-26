#pragma once
/// @file rpm_notch.hpp
/// @brief RPM-tracked notch filters at the rotor harmonics, for the 3-axis gyro.
///
/// Motor and propeller vibration sits at the rotor rotation frequency and its
/// harmonics, which move with the throttle. Given the rotor speeds (from the
/// ESC telemetry on hardware, or the model in SITL), RpmNotch places a notch at
/// each harmonic h·ω/2π of each rotor and runs every axis of the gyro through
/// all of them in series.
///
///  - A centre below min_hz is clamped to min_hz, and its notch fades out
///    linearly from full depth at min_hz + fade_hz to none at min_hz: at low
///    throttle a notch would otherwise sit in the control band.
///  - A centre above f_max = 0.45·fs is out of the section's design range:
///    that notch is parked at f_max with weight 0.
///  - Each notch's output is blended as y ← y + w·(N(y) − y). Every section
///    updates even at w = 0, so its history is live when it fades back in.
///
/// Model
/// -----
///   RpmNotch<T, Rotors, MaxHarmonics>
///     cfg_    : RpmNotchConfig<T> [owned, immutable]
///     notch_  : Biquad<T> [Rotors][MaxHarmonics][3] — one per rotor, harmonic, axis;
///               the three axes of a notch share coefficients
///     center_ : T [Rotors][MaxHarmonics] — centre frequency [Hz]
///     weight_ : T [Rotors][MaxHarmonics] — blend weight
///
/// Invariants
/// ----------
///   I1: rpm_notch_config_valid(cfg_)                      — constrains cfg_
///   I2: every centre in [min_hz, f_max]                   — constrains center_
///   I3: every weight in [0, 1]                            — constrains weight_
///   (each Biquad keeps its own I1–I2)
///
/// Only the first cfg_.harmonics harmonics are used; the rest stay at the
/// constructed state. The operation count of apply() depends only on the
/// configuration; update_rotor_speeds() has two arms per notch (in range, or
/// above f_max) and a hold arm per rotor.
///
/// Validity envelope and non-goals: assumes the vibration lies at integer
/// harmonics of the rotor rate, which flight logs must confirm (F3). It does
/// not estimate the frequencies itself (no FFT tracking) and does not handle
/// aliasing of harmonics above fs/2.
///
/// Resources: no allocation, no exceptions.

#include <branes/reflex/dsp/biquad.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace branes::reflex::dsp {

/// Configuration of an RpmNotch. The defaults are common flight-controller
/// settings for a 5-inch-class quadrotor at a 1 kHz loop; set sample_rate to
/// the rate apply() is called at.
template <typename T>
struct RpmNotchConfig {
    T sample_rate{1000};    ///< rate of apply() [Hz]
    unsigned harmonics{3};  ///< harmonics per rotor, 1…MaxHarmonics
    T q{5};                 ///< quality factor of every notch
    T min_hz{100};          ///< lowest centre frequency [Hz]
    T fade_hz{50};          ///< fade-in range above min_hz [Hz]; 0 switches on at min_hz
};

/// Highest centre frequency for a sample rate: 0.45·fs.
template <typename T>
[[nodiscard]] T rpm_notch_max_hz(T sample_rate) noexcept {
    return T(0.45) * sample_rate;
}

/// True if `c` satisfies invariant I1 of RpmNotch<T, ·, MaxHarmonics>:
/// finite, fs > 0, 1 ≤ harmonics ≤ MaxHarmonics, q > 0, and
/// 0 < min_hz ≤ min_hz + fade_hz < f_max.
template <typename T, std::size_t MaxHarmonics>
[[nodiscard]] bool rpm_notch_config_valid(const RpmNotchConfig<T>& c) noexcept {
    using std::isfinite;
    const T zero(0);
    return isfinite(c.sample_rate) && isfinite(c.q) && isfinite(c.min_hz) && isfinite(c.fade_hz) &&
           zero < c.sample_rate && c.harmonics >= 1 && c.harmonics <= MaxHarmonics && zero < c.q && zero < c.min_hz &&
           !(c.fade_hz < zero) && c.min_hz + c.fade_hz < rpm_notch_max_hz(c.sample_rate);
}

/// RPM-tracked notches for a 3-axis signal. See the file comment for the
/// model and invariants.
template <typename T, std::size_t Rotors = 4, std::size_t MaxHarmonics = 3>
class RpmNotch {
public:
    static_assert(Rotors >= 1 && MaxHarmonics >= 1);

    /// Pre:  rpm_notch_config_valid<T, MaxHarmonics>(cfg).
    /// Post: every notch at min_hz with weight 0 (pass-through until the
    ///       first rotor speeds arrive), zero history. Establishes I1–I3.
    explicit RpmNotch(const RpmNotchConfig<T>& cfg) noexcept : cfg_{cfg} {
        assert((rpm_notch_config_valid<T, MaxHarmonics>(cfg_)));  // Pre / I1
        const BiquadCoefficients<T> c = notch(cfg_.min_hz, cfg_.q, cfg_.sample_rate);
        for (std::size_t r = 0; r < Rotors; ++r) {
            for (std::size_t h = 0; h < MaxHarmonics; ++h) {
                center_[r][h] = cfg_.min_hz;
                weight_[r][h] = T(0);
                for (Biquad<T>& b : notch_[r][h]) {
                    b = Biquad<T>(c);
                }
            }
        }
        assert(check_invariant());
    }

    /// Retune every notch to the rotor speeds `omega` [rad/s].
    ///
    /// Pre:  none on the arguments. A non-finite or negative speed is a
    ///       telemetry fault: that rotor keeps its previous notches.
    /// Post: for each rotor with a valid speed and each harmonic h = 1…H, with
    ///       f = h·ω/2π: if f ≤ f_max, centre = max(f, min_hz) and weight =
    ///       clamp((f − min_hz)/fade_hz, 0, 1) (a step at min_hz if fade_hz = 0);
    ///       otherwise centre = f_max and weight = 0. I1–I3 hold.
    void update_rotor_speeds(const std::array<T, Rotors>& omega) noexcept {
        using std::isfinite;
        const T f_max = rpm_notch_max_hz(cfg_.sample_rate);
        for (std::size_t r = 0; r < Rotors; ++r) {
            if (!isfinite(omega[r]) || omega[r] < T(0)) {
                continue;  // fault: hold this rotor's notches
            }
            const T fundamental = omega[r] / (T(2) * T(std::numbers::pi));
            for (std::size_t h = 0; h < cfg_.harmonics; ++h) {
                const T f = T(static_cast<double>(h + 1)) * fundamental;
                T center = f;
                T weight = T(0);
                if (f_max < f) {
                    center = f_max;  // out of range: parked, off (I2, I3)
                } else {
                    if (center < cfg_.min_hz) {
                        center = cfg_.min_hz;  // I2
                    }
                    if (T(0) < cfg_.fade_hz) {
                        weight = (f - cfg_.min_hz) / cfg_.fade_hz;
                    } else {
                        weight = f < cfg_.min_hz ? T(0) : T(1);
                    }
                    weight = weight < T(0) ? T(0) : (T(1) < weight ? T(1) : weight);  // I3
                }
                center_[r][h] = center;
                weight_[r][h] = weight;
                const BiquadCoefficients<T> c = notch(center, cfg_.q, cfg_.sample_rate);
                for (Biquad<T>& b : notch_[r][h]) {
                    b.set_coefficients(c);
                }
            }
        }
        assert(check_invariant());
    }

    /// Filter one 3-axis sample through every active notch in series.
    ///
    /// Pre: none (a non-finite component is held by each section, see Biquad).
    /// Post: returns the filtered sample. I1–I3 hold.
    [[nodiscard]] Vec3<T> apply(const Vec3<T>& x) noexcept {
        std::array<T, 3> y{x(0), x(1), x(2)};
        for (std::size_t r = 0; r < Rotors; ++r) {
            for (std::size_t h = 0; h < cfg_.harmonics; ++h) {
                const T w = weight_[r][h];
                for (std::size_t i = 0; i < 3; ++i) {
                    y[i] = y[i] + w * (notch_[r][h][i].update(y[i]) - y[i]);
                }
            }
        }
        return vec3(y[0], y[1], y[2]);
    }

    /// Restart every section's history at zero; keeps centres and weights.
    void reset() noexcept {
        for (auto& rotor : notch_) {
            for (auto& harmonic : rotor) {
                for (Biquad<T>& b : harmonic) {
                    b.reset();
                }
            }
        }
        assert(check_invariant());
    }

    [[nodiscard]] const RpmNotchConfig<T>& config() const noexcept {
        return cfg_;
    }

    /// Centre frequency [Hz] of rotor r's harmonic h + 1.
    [[nodiscard]] T center_hz(std::size_t r, std::size_t h) const noexcept {
        assert(r < Rotors && h < MaxHarmonics);  // Pre
        return center_[r][h];
    }

    /// Blend weight of rotor r's harmonic h + 1.
    [[nodiscard]] T weight(std::size_t r, std::size_t h) const noexcept {
        assert(r < Rotors && h < MaxHarmonics);  // Pre
        return weight_[r][h];
    }

    /// The section filtering axis `i` at rotor r's harmonic h + 1.
    [[nodiscard]] const Biquad<T>& section(std::size_t r, std::size_t h, std::size_t i) const noexcept {
        assert(r < Rotors && h < MaxHarmonics && i < 3);  // Pre
        return notch_[r][h][i];
    }

private:
    [[nodiscard]] bool check_invariant() const noexcept {
        bool ok = rpm_notch_config_valid<T, MaxHarmonics>(cfg_);  // I1
        const T f_max = rpm_notch_max_hz(cfg_.sample_rate);
        for (std::size_t r = 0; r < Rotors; ++r) {
            for (std::size_t h = 0; h < MaxHarmonics; ++h) {
                ok = ok && !(center_[r][h] < cfg_.min_hz) && !(f_max < center_[r][h]);  // I2
                ok = ok && !(weight_[r][h] < T(0)) && !(T(1) < weight_[r][h]);          // I3
            }
        }
        return ok;
    }

    RpmNotchConfig<T> cfg_;
    std::array<std::array<std::array<Biquad<T>, 3>, MaxHarmonics>, Rotors> notch_{};
    std::array<std::array<T, MaxHarmonics>, Rotors> center_{};
    std::array<std::array<T, MaxHarmonics>, Rotors> weight_{};
};

}  // namespace branes::reflex::dsp

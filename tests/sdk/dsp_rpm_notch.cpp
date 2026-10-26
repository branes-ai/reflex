// branes::reflex::dsp::RpmNotch: validator, notch placement and fading,
// telemetry faults, RPM tracking on a synthetic gyro signal, a bounded output
// under fast retuning, and the rate loop's phase margin with the gyro filters
// in the loop (B2 delay budget). Acceptance criteria: reflex#21.

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/dsp/biquad.hpp>
#include <branes/reflex/dsp/rpm_notch.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include "dsp_harness.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <random>
#include <type_traits>
#include <vector>

using namespace branes::reflex;
using namespace branes::reflex::dsp;
using dsp_harness::kPi;
using dsp_harness::norms;

namespace {

constexpr double kFs = 8000.0;  // gyro rate [Hz]

template <typename T>
RpmNotchConfig<T> gyro_config() {
    RpmNotchConfig<T> c;
    c.sample_rate = T(kFs);
    return c;
}

template <typename T>
std::array<T, 4> speeds(double w0, double w1, double w2, double w3) {
    return {T(w0), T(w1), T(w2), T(w3)};
}

double hz_to_rad(double f) {
    return 2 * kPi * f;
}

}  // namespace

// ── Validator and placement ───────────────────────────────────────────────

TEST_CASE("rpm_notch_config_valid", "[dsp][rpm-notch]") {
    RpmNotchConfig<double> c = gyro_config<double>();
    REQUIRE(rpm_notch_config_valid<double, 3>(c));
    REQUIRE(rpm_notch_config_valid<double, 3>(RpmNotchConfig<double>{}));  // the 1 kHz defaults
    auto bad = [&](auto&& edit) {
        RpmNotchConfig<double> e = c;
        edit(e);
        return !rpm_notch_config_valid<double, 3>(e);
    };
    REQUIRE(bad([](auto& e) { e.harmonics = 0; }));
    REQUIRE(bad([](auto& e) { e.harmonics = 4; }));
    REQUIRE(bad([](auto& e) { e.q = 0; }));
    REQUIRE(bad([](auto& e) { e.min_hz = 0; }));
    REQUIRE(bad([](auto& e) { e.fade_hz = -1; }));
    REQUIRE(bad([](auto& e) { e.min_hz = 3560; }));  // min + fade above 0.45·fs
    REQUIRE(bad([](auto& e) { e.sample_rate = 0; }));
    REQUIRE(bad([](auto& e) { e.q = std::numeric_limits<double>::quiet_NaN(); }));
    REQUIRE_FALSE(bad([](auto& e) { e.fade_hz = 0; }));
}

TEMPLATE_TEST_CASE("a new RpmNotch passes its input through unchanged", "[dsp][rpm-notch]", double, float) {
    using T = TestType;
    RpmNotch<T> n(gyro_config<T>());
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t h = 0; h < 3; ++h) {
            REQUIRE(n.center_hz(r, h) == T(100));
            REQUIRE(n.weight(r, h) == T(0));
        }
    }
    std::mt19937 rng(20261026);
    std::uniform_real_distribution<double> u(-5, 5);
    for (int k = 0; k < 200; ++k) {
        const Vec3<T> x = vec3(T(u(rng)), T(u(rng)), T(u(rng)));
        const Vec3<T> y = n.apply(x);
        REQUIRE(y(0) == x(0));
        REQUIRE(y(1) == x(1));
        REQUIRE(y(2) == x(2));
    }
}

TEST_CASE("update_rotor_speeds places, clamps, fades, and parks the notches", "[dsp][rpm-notch]") {
    RpmNotch<double> n(gyro_config<double>());  // min 100 Hz, fade 50 Hz, f_max 3600 Hz
    n.update_rotor_speeds(speeds<double>(hz_to_rad(264), hz_to_rad(60), hz_to_rad(110), hz_to_rad(1500)));
    // Rotor 0: 264, 528, 792 Hz, all at full depth.
    for (std::size_t h = 0; h < 3; ++h) {
        REQUIRE(std::abs(n.center_hz(0, h) - 264.0 * static_cast<double>(h + 1)) < 1e-9);
        REQUIRE(n.weight(0, h) == 1.0);
    }
    // Rotor 1: 60 Hz is clamped to min_hz and off; 120 Hz fades at 0.4; 180 Hz full.
    REQUIRE(n.center_hz(1, 0) == 100.0);
    REQUIRE(n.weight(1, 0) == 0.0);
    REQUIRE(std::abs(n.center_hz(1, 1) - 120.0) < 1e-9);
    REQUIRE(std::abs(n.weight(1, 1) - 0.4) < 1e-12);
    REQUIRE(n.weight(1, 2) == 1.0);
    // Rotor 2: 110 Hz fades at 0.2.
    REQUIRE(std::abs(n.weight(2, 0) - 0.2) < 1e-12);
    // Rotor 3: 1500 and 3000 Hz in range; 4500 Hz is parked at f_max, off.
    REQUIRE(n.weight(3, 1) == 1.0);
    REQUIRE(n.center_hz(3, 2) == rpm_notch_max_hz(kFs));
    REQUIRE(n.weight(3, 2) == 0.0);
    // The sections carry the notch at their centre.
    const BiquadCoefficients<double> c = notch(n.center_hz(0, 1), 5.0, kFs);
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(n.section(0, 1, i).coefficients().b1 == c.b1);
    }
}

TEST_CASE("fade_hz = 0 switches each notch on at min_hz", "[dsp][rpm-notch]") {
    RpmNotchConfig<double> c = gyro_config<double>();
    c.fade_hz = 0;
    RpmNotch<double> n(c);
    n.update_rotor_speeds(speeds<double>(hz_to_rad(99), hz_to_rad(100), hz_to_rad(101), 0));
    REQUIRE(n.weight(0, 0) == 0.0);
    REQUIRE(n.weight(1, 0) == 1.0);
    REQUIRE(n.weight(2, 0) == 1.0);
    REQUIRE(n.weight(3, 0) == 0.0);  // ω = 0: clamped, off
}

TEST_CASE("only the configured harmonics move", "[dsp][rpm-notch]") {
    RpmNotchConfig<double> c = gyro_config<double>();
    c.harmonics = 2;
    RpmNotch<double> n(c);
    n.update_rotor_speeds(speeds<double>(hz_to_rad(300), hz_to_rad(300), hz_to_rad(300), hz_to_rad(300)));
    REQUIRE(n.weight(0, 1) == 1.0);
    REQUIRE(n.center_hz(0, 2) == 100.0);
    REQUIRE(n.weight(0, 2) == 0.0);
}

TEMPLATE_TEST_CASE("a non-finite or negative rotor speed holds that rotor's notches",
                   "[dsp][rpm-notch]",
                   double,
                   float) {
    using T = TestType;
    RpmNotch<T> n(gyro_config<T>());
    n.update_rotor_speeds(speeds<T>(hz_to_rad(250), hz_to_rad(260), hz_to_rad(270), hz_to_rad(280)));
    const T before[4] = {n.center_hz(0, 0), n.center_hz(1, 0), n.center_hz(2, 0), n.center_hz(3, 0)};
    n.update_rotor_speeds(
        {std::numeric_limits<T>::quiet_NaN(), T(-1), std::numeric_limits<T>::infinity(), T(hz_to_rad(300))});
    REQUIRE(n.center_hz(0, 0) == before[0]);
    REQUIRE(n.center_hz(1, 0) == before[1]);
    REQUIRE(n.center_hz(2, 0) == before[2]);
    REQUIRE(n.center_hz(3, 0) != before[3]);
    REQUIRE(std::abs(static_cast<double>(n.center_hz(3, 0)) - 300.0) < 1e-3);
}

TEMPLATE_TEST_CASE("apply holds an axis's last output on a non-finite input", "[dsp][rpm-notch]", double, float) {
    using T = TestType;
    // The axes are independent filters. Axis 1 faults now and then: it must
    // return its last output and continue as if the fault never came (as
    // `axis1`, which never sees it). Axes 0 and 2 run on (as `others`).
    RpmNotch<T> n(gyro_config<T>());
    RpmNotch<T> axis1(gyro_config<T>());
    RpmNotch<T> others(gyro_config<T>());
    const auto omega = speeds<T>(hz_to_rad(250), hz_to_rad(260), hz_to_rad(270), hz_to_rad(280));
    n.update_rotor_speeds(omega);
    axis1.update_rotor_speeds(omega);
    others.update_rotor_speeds(omega);
    std::mt19937 rng(20261026);
    std::uniform_real_distribution<double> u(-1, 1);
    const T bad[] = {
        std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::infinity(), -std::numeric_limits<T>::infinity()};
    T last1{0};
    for (int k = 0; k < 600; ++k) {
        const Vec3<T> x = vec3(T(u(rng)), T(u(rng)), T(u(rng)));
        const bool fault = k % 40 == 20;
        const T x1 = fault ? bad[(k / 40) % 3] : x(1);
        const Vec3<T> y = n.apply(vec3(x(0), x1, x(2)));
        const Vec3<T> ref = others.apply(vec3(x(0), T(0), x(2)));
        REQUIRE(y(0) == ref(0));
        REQUIRE(y(2) == ref(2));
        if (fault) {
            REQUIRE(y(1) == last1);
        } else {
            REQUIRE(y(1) == axis1.apply(vec3(T(0), x(1), T(0)))(1));
            last1 = y(1);
        }
    }
}

TEST_CASE("apply holds an axis's last output when a blend overflows", "[dsp][rpm-notch]") {
    using T = float;
    RpmNotch<T> n(gyro_config<T>());
    n.update_rotor_speeds(speeds<T>(hz_to_rad(250), hz_to_rad(260), hz_to_rad(270), hz_to_rad(280)));
    // Alternating near-maximum samples drive N(y) − y past the range of float.
    const T big = T(0.9) * std::numeric_limits<T>::max();
    T previous = n.apply(vec3(T(0), T(0), T(0)))(0);
    int held = 0;
    for (int k = 0; k < 64; ++k) {
        const T x = k % 2 == 0 ? big : -big;
        const T y = n.apply(vec3(x, T(0), T(0)))(0);
        REQUIRE(std::isfinite(y));
        held += y == previous ? 1 : 0;
        previous = y;
    }
    INFO("held samples " << held);
    REQUIRE(held > 0);
}

// ── RPM tracking on a synthetic gyro signal ───────────────────────────────

namespace {

struct Tracking {
    double vibration_db;  ///< residual vibration RMS / input vibration RMS
    double motion_gain;   ///< measured |H| of the 5 Hz motion
};

/// 5 Hz body motion (0.5 rad/s) plus 0.1 rad/s at harmonics 1–3 of four
/// rotors spread ±5 % around hover, ramped from hover to 1.5× hover over 1 s
/// after 0.1 s at hover. The notches get the rotor speeds times
/// (1 + speed_error). The filters are linear (time-varying), so running the
/// motion alone through an identical filter separates the vibration residual
/// from the motion.
template <typename T>
Tracking track(double speed_error) {
    const double hover = model::QuadrotorModel<double>(model::QuadrotorParams<double>{}).hover_rotor_speed();
    constexpr std::array<double, 4> spread = {0.95, 1 - 0.05 / 3, 1 + 0.05 / 3, 1.05};
    constexpr double kMotionHz = 5.0;
    constexpr double kMotionAmp = 0.5;
    constexpr double kVibrationAmp = 0.1;
    const std::size_t hold = static_cast<std::size_t>(0.1 * kFs);
    const std::size_t ramp = static_cast<std::size_t>(1.0 * kFs);

    RpmNotch<T> with(gyro_config<T>());
    RpmNotch<T> motion_only(gyro_config<T>());
    std::array<std::array<double, 3>, 4> phase{};
    double vib_in = 0, vib_out = 0, motion_i = 0, motion_q = 0;
    for (std::size_t k = 0; k < hold + ramp; ++k) {
        const double s = k < hold ? 1.0 : 1.0 + 0.5 * static_cast<double>(k - hold) / static_cast<double>(ramp);
        std::array<T, 4> omega{};
        double vibration = 0;
        for (std::size_t r = 0; r < 4; ++r) {
            const double w = hover * spread[r] * s;
            omega[r] = T(w * (1 + speed_error));
            for (std::size_t h = 0; h < 3; ++h) {
                vibration += kVibrationAmp * std::sin(phase[r][h]);
                phase[r][h] += static_cast<double>(h + 1) * w / kFs;
            }
        }
        with.update_rotor_speeds(omega);
        motion_only.update_rotor_speeds(omega);
        const double theta = 2 * kPi * kMotionHz * static_cast<double>(k) / kFs;
        const double motion = kMotionAmp * std::sin(theta);
        const double a = static_cast<double>(with.apply(vec3(T(motion + vibration), T(0), T(0)))(0));
        const double b = static_cast<double>(motion_only.apply(vec3(T(motion), T(0), T(0)))(0));
        if (k >= hold) {  // the ramp: exactly five motion periods
            vib_in += vibration * vibration;
            vib_out += (a - b) * (a - b);
            motion_i += b * std::sin(theta);
            motion_q += b * std::cos(theta);
        }
    }
    const double scale = 2.0 / static_cast<double>(ramp);
    return {10 * std::log10(vib_out / vib_in), std::hypot(motion_i, motion_q) * scale / kMotionAmp};
}

}  // namespace

TEMPLATE_TEST_CASE("RPM tracking: vibration residual <= -30 dB, motion amplitude within 2 %",
                   "[dsp][rpm-notch][verification]",
                   double,
                   float) {
    using T = TestType;
    const Tracking t = track<T>(0.0);
    std::printf("rpm notch tracking (%s): vibration residual %6.1f dB, motion gain %.4f\n",
                std::is_same_v<T, double> ? "double" : "float",
                t.vibration_db,
                t.motion_gain);
    REQUIRE(t.vibration_db <= -30.0);
    REQUIRE(std::abs(t.motion_gain - 1.0) <= 0.02);
}

TEST_CASE("RPM tracking with a 2 % speed estimate error, reported", "[dsp][rpm-notch][characterization]") {
    for (const double e : {-0.02, 0.02}) {
        const Tracking t = track<double>(e);
        std::printf("rpm notch, speed error %+.0f %%: vibration residual %6.1f dB, motion gain %.4f\n",
                    e * 100,
                    t.vibration_db,
                    t.motion_gain);
    }
    SUCCEED();
}

// ── Fast retuning keeps the output bounded ────────────────────────────────

TEMPLATE_TEST_CASE("a sweep at motor-acceleration rates stays within the worst-case gain bound",
                   "[dsp][rpm-notch][property]",
                   double,
                   float) {
    using T = TestType;
    const RpmNotchConfig<T> cfg = gyro_config<T>();
    const double f_max = rpm_notch_max_hz(kFs);
    // Each blended stage has gain ≤ (1 − w) + w·‖h‖₁ ≤ max over centres of
    // ‖h‖₁ (≥ 1); twelve stages in series multiply. The maximum is taken
    // over a 1 Hz grid of centres.
    double worst_l1 = 1.0;
    for (double f = 100.0; f <= f_max; f += 1.0) {
        worst_l1 = std::max(worst_l1, norms(dsp_harness::to_double(notch(T(f), cfg.q, T(kFs)))).l1);
    }
    const double bound = std::pow(worst_l1, 12);

    const model::QuadrotorParams<double> p;
    const double w_max = model::QuadrotorModel<double>(p).max_rotor_speed();
    // Full-range swings within one motor time constant, each rotor offset.
    const double period = 2 * p.motor_time_constant;
    RpmNotch<T> n(cfg);
    std::mt19937 rng(20261026);
    std::uniform_real_distribution<double> u(-1, 1);
    double worst = 0;
    for (int k = 0; k < static_cast<int>(kFs); ++k) {
        const double t = k / kFs;
        std::array<T, 4> omega{};
        for (std::size_t r = 0; r < 4; ++r) {
            const double phase = 2 * kPi * (t / period + 0.25 * static_cast<double>(r));
            omega[r] = T(w_max * (0.65 + 0.35 * std::sin(phase)));
        }
        n.update_rotor_speeds(omega);
        const Vec3<T> y = n.apply(vec3(T(u(rng)), T(u(rng)), T(u(rng))));
        worst = std::max({worst, std::abs(double(y(0))), std::abs(double(y(1))), std::abs(double(y(2)))});
    }
    INFO("worst |y| " << worst << ", bound " << bound << " (per-notch ‖h‖₁ ≤ " << worst_l1 << ")");
    REQUIRE(worst <= bound);
}

// ── B2 delay budget: phase margin with the filters in the rate loop ───────

TEST_CASE("rate loop: analytic phase margin >= 45 deg with the gyro filters in the loop", "[dsp][rpm-notch][budget]") {
    // The rate loop of cascade_closed_loop.cpp's margin test,
    //   L(s) = (k_p + k_i/s + k_d·s) / (s (τ s + 1)) · e^(−s T_d),
    // times the gyro filter chain F(e^{jωT}) at 8 kHz: a Butterworth low-pass
    // at 16× the crossover (≤ 5° lag there: ∠ ≈ −√2·ω/ω_c) and three-harmonic
    // RPM notches at hover speed on all four rotors.
    const model::QuadrotorParams<double> p;
    const auto c = control::cascade_config_from(p).rate.axis[0];
    const double tau = p.motor_time_constant;
    const double delay = 1e-3;
    const auto plant = [&](double w) {
        const std::complex<double> s(0.0, w);
        return (c.kp + c.ki / s + c.kd * s) / (s * (tau * s + 1.0)) * std::exp(-s * delay);
    };
    const auto crossover = [](auto&& loop_gain) {
        double lo = 1e-2, hi = 1e4;  // |L| falls through 1 once in this range
        for (int i = 0; i < 200; ++i) {
            const double mid = std::sqrt(lo * hi);
            (std::abs(loop_gain(mid)) > 1.0 ? lo : hi) = mid;
        }
        return lo;
    };
    const double wc_bare = crossover(plant);
    const double lpf_hz = 16 * wc_bare / (2 * kPi);
    const BiquadCoefficients<double> lpf = butterworth_lowpass(lpf_hz, kFs);
    const double hover_hz = model::QuadrotorModel<double>(p).hover_rotor_speed() / (2 * kPi);
    const auto filters = [&](double w) {
        const double f = w / (2 * kPi);
        std::complex<double> h = response(lpf, f, kFs);
        for (int r = 0; r < 4; ++r) {
            for (int k = 1; k <= 3; ++k) {
                h *= response(notch(k * hover_hz, 5.0, kFs), f, kFs);
            }
        }
        return h;
    };
    const auto filtered = [&](double w) { return plant(w) * filters(w); };
    const double wc = crossover(filtered);
    const double bare_pm = 180.0 + std::arg(plant(wc_bare)) * 180.0 / kPi;
    const double pm = 180.0 + std::arg(filtered(wc)) * 180.0 / kPi;
    const double lag_deg = -std::arg(filters(wc)) * 180.0 / kPi;
    std::printf("rate loop: crossover %.2f Hz, gyro LPF %.0f Hz; phase margin %.1f deg bare, %.1f deg filtered "
                "(filters lag %.2f deg, %.3f ms group delay at crossover)\n",
                wc / (2 * kPi),
                lpf_hz,
                bare_pm,
                pm,
                lag_deg,
                group_delay(lpf, wc / (2 * kPi), kFs) * 1e3);
    REQUIRE(pm >= 45.0);
}

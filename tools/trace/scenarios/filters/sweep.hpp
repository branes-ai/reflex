#pragma once

// A throttle sweep through RPM-tracked notches, at the 8 kHz gyro rate.
//
// The gyro measures a 5 Hz roll rate of 0.5 rad/s, the vibration of four
// rotors spread ±5 % at their first three harmonics, and sensor noise
// (filters_trace.hpp). The rotors idle for 0.5 s with the fleet's
// fundamental near 70 Hz, are swept linearly to 90 % of their maximum speed
// (fundamentals near 525 Hz, third harmonics near 1.58 kHz) over 3 s, and
// hold there for 0.5 s. Three filters see the same signal:
//  - the tracked notches: an RpmNotch (the default configuration at 8 kHz:
//    three harmonics, min_hz 100, fade_hz 50) given the rotor speeds every
//    sample, scaled by (1 + speed error) as a telemetry error would;
//  - fixed notches: the same RpmNotch given the hover speeds once, so its
//    notches stay where hover puts them;
//  - nothing, for the raw spectrum.
// Below min_hz the tracked notches are off (they would sit in the control
// band), and they fade in over the next 50 Hz. Once every fundamental is past
// 150 Hz all twelve notches are on and the vibration they leave is about
// −79 dB. The fixed notches remove the vibration only while the rotors pass
// hover.
//
// The filters are linear, so a copy of each bank run on the motion (with the
// sensor noise, which no notch removes) separates the vibration it leaves
// from the rest, as in the RPM tracking test (tests/sdk/dsp_rpm_notch.cpp).
//
// Checks:
//  - BB-I2: the vibration left by the tracked notches, over the part of the
//    sweep where every fundamental is past min_hz + fade_hz, is at most
//    −30 dB of the vibration in (the RPM tracking criterion of reflex#21).
//  - BB-C2: the 5 Hz motion passes with its amplitude within 2 % (the same
//    criterion), measured over the ramp's 15 motion periods.
//
// The parameters are the notches' quality factor and the speed error.
// sweep.cpp writes the trace at the defaults (q = 5, no error); the
// explainer's live widget runs this function, compiled to WebAssembly, at
// the reader's values.

#include "filters_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace branes::reflex::trace::scenarios::filters_sweep {

/// The widget's parameters, with the ranges the sliders offer.
struct Params {
    double q = 5;            ///< quality factor of every notch, in [2, 10]
    double speed_error = 0;  ///< relative error of the speeds the notches get, in [−0.1, 0.1]
};

namespace trace = branes::reflex::trace;
namespace ft = branes::reflex::trace::filters;
namespace dsp = branes::reflex::dsp;

inline constexpr double kIdle = 0.5;         // [s]
inline constexpr double kRamp = 3.0;         // [s]
inline constexpr double kHold = 0.5;         // [s]
inline constexpr double kIdleHz = 70;        // fleet fundamental at idle [Hz]
inline constexpr double kTopFraction = 0.9;  // of the maximum rotor speed
inline constexpr double kMotionHz = 5;
inline constexpr double kMotionAmp = 0.5;

/// A sweep's gyro rate and spectrum: 25 ms per trace sample, and 31.25 Hz bins
/// from a 32 ms window.
struct Setup {
    double fs;             ///< gyro rate [Hz]
    std::size_t hop;       ///< gyro samples per trace sample
    std::size_t window;    ///< spectrum window [samples]
    std::size_t bins;      ///< spectrum bins, from 0 Hz
    const char* scenario;  ///< the source that writes the trace
};

/// The explainer's sweep: 8 kHz, spectrum to 1.75 kHz.
inline constexpr Setup kGyro8k{8000, 200, 256, 57, "tools/trace/scenarios/filters/sweep.cpp"};
/// The deployment guide's: the default 1 kHz configuration, spectrum to Nyquist.
inline constexpr Setup kGyro1k{1000, 25, 32, 17, "tools/trace/scenarios/filters/low_rate.cpp"};

inline trace::Trace sweep(const Params& p, const Setup& setup) {
    if (!(p.q >= 2 && p.q <= 10) || !(p.speed_error >= -0.1 && p.speed_error <= 0.1)) {
        throw std::invalid_argument("filters_sweep: parameters outside the widget's ranges");
    }
    const double fs = setup.fs;
    const std::size_t kHop = setup.hop;
    const std::size_t kBins = setup.bins;
    const double idle = 2 * ft::kPi * kIdleHz;
    const double top = kTopFraction * ft::max_speed();
    const double hover = ft::hover_speed();
    const auto fleet = [&](double t) {
        if (t < kIdle) {
            return idle;
        }
        return idle + (top - idle) * std::min(1.0, (t - kIdle) / kRamp);
    };

    dsp::RpmNotchConfig<double> cfg;
    cfg.sample_rate = fs;
    cfg.q = p.q;
    dsp::RpmNotch<double> tracked(cfg);
    dsp::RpmNotch<double> tracked_motion(cfg);
    dsp::RpmNotch<double> fixed(cfg);
    dsp::RpmNotch<double> fixed_motion(cfg);
    fixed.update_rotor_speeds(ft::rotor_speeds(hover));
    fixed_motion.update_rotor_speeds(ft::rotor_speeds(hover));
    ft::Vibration vibration(fs);
    ft::Spectrum raw_spectrum(fs, setup.window, kBins);
    ft::Spectrum filtered_spectrum(fs, setup.window, kBins);

    const std::size_t total = static_cast<std::size_t>(std::lround((kIdle + kRamp + kHold) * fs));
    trace::Trace tr({.commit = {},
                     .block = "dsp/filters",
                     .scenario = setup.scenario,
                     .seed = 20261030,
                     .compiler = {},
                     .dt_sim = 1 / fs,
                     .dt_trace = static_cast<double>(kHop) / fs});
    tr.channel("rotor_hz", 4, "Hz");
    tr.channel("raw_spectrum", kBins, "rad/s");
    tr.channel("filtered_spectrum", kBins, "rad/s");
    tr.channel("spectrum_bin_hz", 1, "Hz");
    tr.channel("notch_hz", 12, "Hz");
    tr.channel("notch_weight", 12, "1");
    tr.channel("vibration_db", 2, "dB");
    tr.channel("vibration_limit", 1, "dB");

    ft::Energy hop_in, hop_fixed, hop_tracked;  // this trace sample's vibration
    ft::Energy all_in, all_tracked;             // every notch on (BB-I2)
    double motion_i = 0;
    double motion_q = 0;
    std::vector<double> centers;
    std::vector<double> weights;
    const double on_hz = cfg.min_hz + cfg.fade_hz;
    double on_at = -1;
    double fade_at = -1;
    double hover_at = -1;
    std::array<double, ft::kHarmonics> parked_at{-1, -1, -1};  // per harmonic, the fastest rotor's

    const auto record = [&](double t, const std::array<double, 4>& omega) {
        tr.sample(t);
        tr.set(
            "rotor_hz",
            {omega[0] / (2 * ft::kPi), omega[1] / (2 * ft::kPi), omega[2] / (2 * ft::kPi), omega[3] / (2 * ft::kPi)});
        tr.set("raw_spectrum", raw_spectrum.amplitudes());
        tr.set("filtered_spectrum", filtered_spectrum.amplitudes());
        tr.set("spectrum_bin_hz", raw_spectrum.bin_hz());
        ft::notch_state(tracked, centers, weights);
        tr.set("notch_hz", centers);
        tr.set("notch_weight", weights);
        tr.set("vibration_db", {ft::ratio_db(hop_fixed, hop_in), ft::ratio_db(hop_tracked, hop_in)});
        tr.set("vibration_limit", -30.0);
        hop_in = hop_fixed = hop_tracked = {};
    };

    record(0, ft::rotor_speeds(fleet(0)));
    for (std::size_t k = 0; k < total; ++k) {
        const double t = static_cast<double>(k) / fs;
        const std::array<double, 4> omega = ft::rotor_speeds(fleet(t));
        std::array<double, 4> reported{};
        for (std::size_t r = 0; r < 4; ++r) {
            reported[r] = omega[r] * (1 + p.speed_error);
        }
        tracked.update_rotor_speeds(reported);
        tracked_motion.update_rotor_speeds(reported);

        const double theta = 2 * ft::kPi * kMotionHz * t;
        const double motion = kMotionAmp * std::sin(theta) + vibration.noise();
        const double vib = vibration.next(omega);
        const double gyro = motion + vib;
        const double y = tracked.apply(vec3(gyro, 0.0, 0.0))(0);
        const double y_motion = tracked_motion.apply(vec3(motion, 0.0, 0.0))(0);
        const double z = fixed.apply(vec3(gyro, 0.0, 0.0))(0) - fixed_motion.apply(vec3(motion, 0.0, 0.0))(0);
        raw_spectrum.push(gyro);
        filtered_spectrum.push(y);

        hop_in.add(vib);
        hop_fixed.add(z);
        hop_tracked.add(y - y_motion);
        const double slowest_hz = omega[0] / (2 * ft::kPi);
        if (slowest_hz >= on_hz) {
            all_in.add(vib);
            all_tracked.add(y - y_motion);
            on_at = on_at < 0 ? t : on_at;
        }
        fade_at = fade_at < 0 && slowest_hz >= cfg.min_hz ? t : fade_at;
        hover_at = hover_at < 0 && fleet(t) >= hover ? t : hover_at;
        for (std::size_t h = 0; h < ft::kHarmonics; ++h) {  // above 0.45·fs, a notch parks there, off
            const double f = static_cast<double>(h + 1) * omega[3] / (2 * ft::kPi);
            parked_at[h] = parked_at[h] < 0 && f > dsp::rpm_notch_max_hz(fs) ? t : parked_at[h];
        }
        if (t >= kIdle && t < kIdle + kRamp) {  // noise and all: it averages out over the ramp
            motion_i += y_motion * std::sin(theta);
            motion_q += y_motion * std::cos(theta);
        }
        if ((k + 1) % kHop == 0) {
            record(static_cast<double>(k + 1) / fs, omega);
        }
    }

    const double ramp_samples = kRamp * fs;
    const double motion_gain = std::hypot(motion_i, motion_q) * 2 / ramp_samples / kMotionAmp;
    const bool all_on = parked_at[2] < 0 || parked_at[2] > on_at;
    std::vector<std::pair<double, std::string>> events{
        {kIdle, "throttle up: every fundamental is below min_hz (100 Hz), so those notches are off"},
        {fade_at, "the slowest rotor's fundamental reaches 100 Hz: its notch starts to fade in"},
        {on_at,
         all_on ? "every fundamental past 150 Hz: all twelve notches on"
                : "every fundamental past 150 Hz: their notches are fully on"},
        {hover_at, "passing hover, where the fixed notches sit"}};
    constexpr const char* kOrdinal[] = {"the fastest fundamental", "a second harmonic", "a third harmonic"};
    for (std::size_t h = 0; h < ft::kHarmonics; ++h) {
        if (parked_at[h] >= 0) {
            events.emplace_back(parked_at[h],
                                std::string(kOrdinal[h]) + " passes 0.45·fs: its notch is parked there, off");
        }
    }
    char top_label[160];
    std::snprintf(top_label,
                  sizeof top_label,
                  "held at 90 %% of maximum speed: fundamentals %.0f to %.0f Hz",
                  top * ft::kSpread[0] / (2 * ft::kPi),
                  top * ft::kSpread[3] / (2 * ft::kPi));
    events.emplace_back(kIdle + kRamp, top_label);
    std::sort(events.begin(), events.end());
    for (auto& [t, label] : events) {
        tr.event(t, std::move(label));
    }
    tr.check({.id = "BB-I2",
              .metric = "vibration_residual_db",
              .unit = "dB",
              .value = ft::ratio_db(all_tracked, all_in),
              .limit = -30,
              .comparison = "<="});
    tr.check({.id = "BB-C2",
              .metric = "motion_gain_error",
              .unit = "1",
              .value = std::abs(motion_gain - 1),
              .limit = 0.02,
              .comparison = "<="});
    return tr;
}

/// The explainer's sweep, at 8 kHz.
inline trace::Trace run(const Params& p) {
    return sweep(p, kGyro8k);
}

}  // namespace branes::reflex::trace::scenarios::filters_sweep

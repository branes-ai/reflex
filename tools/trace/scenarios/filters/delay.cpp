// Writes $REFLEX_TRACE_DIR/filters/delay.json: what the gyro filters cost the
// rate loop, as delay.
//
// The gyro chain of the delay-budget test (tests/sdk/dsp_rpm_notch.cpp) at
// 8 kHz: a 2nd-order Butterworth low-pass at 16 times the rate loop's
// crossover (101 Hz), and RPM notches at the hover speeds of four rotors
// spread ±5 % (three harmonics each, the default configuration). A 1 rad/s
// roll-rate step at 5 ms goes through the notches alone, the low-pass alone,
// and the whole chain, sampled every gyro sample for 40 ms. No vibration is
// added: the trace shows the filters' own response.
//
// Check (BB-C2): the rate loop's phase margin with the chain in the loop is
// at least 45°. It is computed as the test computes it, from response() on
// the loop L(s) = (k_p + k_i/s + k_d·s) / (s (τ s + 1)) · e^(−s·1 ms) of
// RP-1's default rate gains, times the chain.

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/dsp/biquad.hpp>

#include "filters_trace.hpp"

#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdio>

int main() {
    namespace trace = branes::reflex::trace;
    namespace qt = branes::reflex::trace::quadrotor;
    namespace ft = branes::reflex::trace::filters;
    namespace dsp = branes::reflex::dsp;
    namespace control = branes::reflex::control;
    namespace model = branes::reflex::model;
    using branes::reflex::vec3;

    return qt::guarded("delay", [] {
        constexpr double kFs = ft::kGyroRate;
        constexpr double kStepAt = 0.005;
        constexpr double kDuration = 0.040;
        constexpr double kDelay = 1e-3;  // command delay in the loop model [s]
        const model::QuadrotorParams<double> p;
        const auto gains = control::cascade_config_from(p).rate.axis[0];
        const double tau = p.motor_time_constant;
        const auto plant = [&](double w) {
            const std::complex<double> s(0.0, w);
            return (gains.kp + gains.ki / s + gains.kd * s) / (s * (tau * s + 1.0)) * std::exp(-s * kDelay);
        };
        const auto crossover = [](auto&& loop) {
            double lo = 1e-2;
            double hi = 1e4;  // |L| falls through 1 once in this range
            for (int i = 0; i < 200; ++i) {
                const double mid = std::sqrt(lo * hi);
                (std::abs(loop(mid)) > 1.0 ? lo : hi) = mid;
            }
            return lo;
        };
        const double wc_bare = crossover(plant);
        const double lpf_hz = 16 * wc_bare / (2 * ft::kPi);
        const dsp::BiquadCoefficients<double> lpf_c = dsp::butterworth_lowpass(lpf_hz, kFs);

        dsp::RpmNotchConfig<double> cfg;
        cfg.sample_rate = kFs;
        dsp::RpmNotch<double> notches(cfg);
        dsp::RpmNotch<double> chain_notches(cfg);
        const std::array<double, 4> hover = ft::rotor_speeds(ft::hover_speed());
        notches.update_rotor_speeds(hover);
        chain_notches.update_rotor_speeds(hover);
        dsp::Biquad<double> lpf(lpf_c);
        dsp::Biquad<double> chain_lpf(lpf_c);

        const auto chain = [&](double w) {
            const double f = w / (2 * ft::kPi);
            std::complex<double> h = dsp::response(lpf_c, f, kFs);
            for (std::size_t r = 0; r < 4; ++r) {
                for (std::size_t k = 0; k < 3; ++k) {
                    h *= dsp::response(dsp::notch(notches.center_hz(r, k), cfg.q, kFs), f, kFs);
                }
            }
            return h;
        };
        const auto filtered = [&](double w) { return plant(w) * chain(w); };
        const double wc = crossover(filtered);
        const double margin = 180.0 + std::arg(filtered(wc)) * 180.0 / ft::kPi;
        const double bare_margin = 180.0 + std::arg(plant(wc_bare)) * 180.0 / ft::kPi;

        trace::Trace tr({.commit = {},
                         .block = "dsp/filters",
                         .scenario = "tools/trace/scenarios/filters/delay.cpp",
                         .seed = 0,
                         .compiler = {},
                         .dt_sim = 1 / kFs,
                         .dt_trace = 1 / kFs});
        tr.channel("roll_rate", 4, "rad/s");

        const std::size_t total = static_cast<std::size_t>(std::lround(kDuration * kFs));
        double half_at = -1;
        for (std::size_t k = 0; k <= total; ++k) {
            const double t = static_cast<double>(k) / kFs;
            const double x = t >= kStepAt ? 1.0 : 0.0;
            const double n = notches.apply(vec3(x, 0.0, 0.0))(0);
            const double l = lpf.update(x);
            const double c = chain_lpf.update(chain_notches.apply(vec3(x, 0.0, 0.0))(0));
            half_at = half_at < 0 && c >= 0.5 ? t : half_at;
            tr.sample(t);
            tr.set("roll_rate", {x, n, l, c});
        }

        char half[160];
        std::snprintf(
            half, sizeof half, "the chain's output reaches half the step, %.1f ms after it", (half_at - kStepAt) * 1e3);
        tr.event(kStepAt, "a 1 rad/s roll-rate step reaches the gyro");
        tr.event(half_at, half);
        tr.check({.id = "BB-C2",
                  .metric = "rate_loop_phase_margin_deg",
                  .unit = "deg",
                  .value = margin,
                  .limit = 45,
                  .comparison = ">="});
        std::printf("delay: crossover %.2f Hz, low-pass %.1f Hz, phase margin %.1f deg bare, %.1f deg filtered\n",
                    wc / (2 * ft::kPi),
                    lpf_hz,
                    bare_margin,
                    margin);
        return qt::write(tr, "filters/delay.json");
    });
}

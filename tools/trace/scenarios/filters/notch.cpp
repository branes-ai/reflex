// Writes $REFLEX_TRACE_DIR/filters/notch.json: what one notch does to a tone
// that sweeps through it.
//
// A single Biquad, notch(f0, q = 5, fs = 8 kHz) with f0 at RP-1's hover
// fundamental (264 Hz), filters a unit-amplitude tone whose frequency rises
// linearly from 50 Hz to 650 Hz over 4 s (150 Hz/s). The gain is measured
// once per trace sample, as the RMS of the output over the RMS of the input
// across the last 25 ms, and recorded beside response(), the gain of the
// design evaluated at the tone's mean frequency over the same 25 ms.
//
// Check (BB-V1): the notch's −3 dB width, measured where the measured gain
// crosses −3 dB on either side of f0 (linear interpolation between trace
// samples), matches the closed form (fs/π)·atan(sin ω0 / 2q) to within the
// 3.75 Hz the tone moves in one trace sample: the sweep cannot resolve an
// edge more finely than that.

#include <branes/reflex/dsp/biquad.hpp>

#include "filters_trace.hpp"

#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

int main() {
    namespace trace = branes::reflex::trace;
    namespace qt = branes::reflex::trace::quadrotor;
    namespace ft = branes::reflex::trace::filters;
    namespace dsp = branes::reflex::dsp;

    return qt::guarded("notch", [] {
        constexpr double kFs = ft::kGyroRate;
        constexpr double kQ = 5;
        constexpr double kStartHz = 50;
        constexpr double kEndHz = 650;
        constexpr double kDuration = 4;
        constexpr std::size_t kHop = 200;     // 25 ms
        constexpr std::size_t kWindow = 256;  // 31.25 Hz bins
        constexpr std::size_t kBins = 33;     // 0 … 1 kHz
        const double f0 = ft::hover_speed() / (2 * ft::kPi);
        const dsp::BiquadCoefficients<double> c = dsp::notch(f0, kQ, kFs);
        dsp::Biquad<double> notch(c);
        ft::Spectrum out_spectrum(kFs, kWindow, kBins);

        trace::Trace tr({.commit = {},
                         .block = "dsp/filters",
                         .scenario = "tools/trace/scenarios/filters/notch.cpp",
                         .seed = 0,
                         .compiler = {},
                         .dt_sim = 1 / kFs,
                         .dt_trace = static_cast<double>(kHop) / kFs});
        tr.channel("tone_hz", 1, "Hz");
        tr.channel("output_spectrum", kBins, "1");
        tr.channel("spectrum_bin_hz", 1, "Hz");
        tr.channel("notch_hz", 1, "Hz");
        tr.channel("gain_db", 2, "dB");

        const auto frequency = [&](double t) { return kStartHz + (kEndHz - kStartHz) * t / kDuration; };
        const auto record = [&](double t, double tone_hz, double measured_db) {
            tr.sample(t);
            tr.set("tone_hz", tone_hz);
            tr.set("output_spectrum", out_spectrum.amplitudes());
            tr.set("spectrum_bin_hz", out_spectrum.bin_hz());
            tr.set("notch_hz", f0);
            tr.set("gain_db", {measured_db, 20 * std::log10(std::abs(dsp::response(c, tone_hz, kFs)))});
        };

        const std::size_t total = static_cast<std::size_t>(std::lround(kDuration * kFs));
        std::vector<double> tone_hz{frequency(0)};
        std::vector<double> gain_db{0};
        record(0, frequency(0), 0);
        ft::Energy in;
        ft::Energy out;
        double phase = 0;
        for (std::size_t k = 0; k < total; ++k) {
            const double t = static_cast<double>(k) / kFs;
            const double x = std::sin(phase);
            phase = std::fmod(phase + 2 * ft::kPi * frequency(t) / kFs, 2 * ft::kPi);
            const double y = notch.update(x);
            out_spectrum.push(y);
            in.add(x);
            out.add(y);
            if ((k + 1) % kHop == 0) {
                const double t_end = static_cast<double>(k + 1) / kFs;
                const double mid_hz = frequency(t_end - 0.5 * static_cast<double>(kHop) / kFs);
                const double db = ft::ratio_db(out, in);
                record(t_end, mid_hz, db);
                tone_hz.push_back(mid_hz);
                gain_db.push_back(db);
                in = out = {};
            }
        }

        // The −3 dB crossings either side of f0, between trace samples.
        double lower = 0;
        double upper = 0;
        for (std::size_t k = 1; k < gain_db.size(); ++k) {
            const double a = gain_db[k - 1] + 3;
            const double b = gain_db[k] + 3;
            const double at = tone_hz[k - 1] + (tone_hz[k] - tone_hz[k - 1]) * a / (a - b);
            if (a > 0 && b <= 0 && lower == 0) {
                lower = at;
            } else if (a <= 0 && b > 0 && lower != 0 && upper == 0) {
                upper = at;
            }
        }
        const double w0 = 2 * ft::kPi * f0 / kFs;
        const double width = kFs / ft::kPi * std::atan(std::sin(w0) / (2 * kQ));
        const double step_hz = (kEndHz - kStartHz) / kDuration * static_cast<double>(kHop) / kFs;

        tr.event(0, "a unit tone at 50 Hz: the notch passes it");
        tr.event((lower - kStartHz) / 150.0, "the tone reaches the notch's lower −3 dB edge");
        tr.event((f0 - kStartHz) / 150.0, "the tone crosses f0 = 264 Hz, where the notch's gain is zero");
        tr.event((upper - kStartHz) / 150.0, "past the upper −3 dB edge: unity gain again");
        tr.check({.id = "BB-V1",
                  .metric = "width_error_hz",
                  .unit = "Hz",
                  .value = std::abs((upper - lower) - width),
                  .limit = step_hz,
                  .comparison = "<="});
        std::printf("notch: f0 %.2f Hz, measured edges %.2f / %.2f Hz, width %.2f Hz, closed form %.2f Hz\n",
                    f0,
                    lower,
                    upper,
                    upper - lower,
                    width);
        return qt::write(tr, "filters/notch.json");
    });
}

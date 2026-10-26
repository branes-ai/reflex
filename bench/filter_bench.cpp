// filter_bench — per-sample latency of the branes::reflex::dsp gyro filters.
//
// Reports, for float and double, the mean time of one Biquad::update, one
// 3-axis RpmNotch::apply (four rotors, three harmonics: 36 sections), and one
// RpmNotch::update_rotor_speeds (twelve notch designs). The input is a
// precomputed noisy signal so no section settles into subnormal numbers.
// These are host latency figures (characterization), not a timing bound.
//
// Usage: filter_bench [iterations]   (default 2,000,000)

#include <branes/reflex/dsp/biquad.hpp>
#include <branes/reflex/dsp/rpm_notch.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using namespace branes::reflex;
using namespace branes::reflex::dsp;

template <typename T, typename F>
double ns_per_call(long iterations, F&& call) {
    T sink{0};
    const auto start = std::chrono::steady_clock::now();
    for (long i = 0; i < iterations; ++i) {
        sink += call(i);
    }
    const auto stop = std::chrono::steady_clock::now();
    if (sink != sink) {
        std::puts("unexpected NaN");
    }
    return std::chrono::duration<double, std::nano>(stop - start).count() / static_cast<double>(iterations);
}

template <typename T>
void report(const char* name, long iterations) {
    constexpr long kSamples = 1024;
    T signal[kSamples];
    for (long i = 0; i < kSamples; ++i) {
        const double t = static_cast<double>(i) / 8000.0;
        signal[i] = T(0.5 * std::sin(31.4 * t) + 0.1 * std::sin(1659.0 * t) + 0.05 * std::sin(9000.0 * t * t));
    }
    Biquad<T> lpf(butterworth_lowpass(T(100), T(8000)));
    const double biquad_ns = ns_per_call<T>(iterations, [&](long i) { return lpf.update(signal[i % kSamples]); });

    RpmNotchConfig<T> cfg;
    cfg.sample_rate = T(8000);
    RpmNotch<T> notches(cfg);
    notches.update_rotor_speeds({T(1580), T(1630), T(1690), T(1740)});
    const double apply_ns = ns_per_call<T>(iterations, [&](long i) {
        const T x = signal[i % kSamples];
        return notches.apply(vec3(x, -x, T(0.5) * x))(0);
    });
    const long retunes = iterations / 10 > 0 ? iterations / 10 : 1;
    const double retune_ns = ns_per_call<T>(retunes, [&](long i) {
        const T w = T(1600) + signal[i % kSamples] * T(400);
        notches.update_rotor_speeds({w, w + T(50), w + T(100), w + T(150)});
        return notches.center_hz(0, 0);
    });
    std::printf("%-6s biquad %6.2f ns | rpm notch apply (36 sections) %7.2f ns | update_rotor_speeds %7.2f ns\n",
                name,
                biquad_ns,
                apply_ns,
                retune_ns);
}

}  // namespace

int main(int argc, char** argv) {
    const long iterations = argc > 1 ? std::strtol(argv[1], nullptr, 10) : 2'000'000L;
    if (iterations <= 0) {
        std::fprintf(stderr, "iterations must be positive\n");
        return 1;
    }
    report<float>("float", iterations);
    report<double>("double", iterations);
    return 0;
}

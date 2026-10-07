// pid_bench — per-update latency of branes::reflex::Pid.
//
// Runs the controller in a closed loop against a trivial plant (so the
// compiler cannot hoist the update out of the loop) and reports the mean time
// per update for float and double. The setpoint is a square wave so the loop
// keeps moving: a loop that settles exactly drives the filtered rate down into
// subnormal numbers, which are an order of magnitude slower and would measure
// the FPU's denormal path rather than the controller. A flight-controller rate loop at 8 kHz has
// a 125 us budget per tick for everything; one PID update should be a tiny
// fraction of that.
//
// Usage: pid_bench [iterations]   (default 10,000,000)

#include <branes/reflex/control/pid.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>

namespace {

template <typename Real>
double ns_per_update(long iterations) {
    branes::reflex::PidConfig<Real> cfg;
    cfg.kp = Real(0.8);
    cfg.ki = Real(2.0);
    cfg.kd = Real(0.02);
    cfg.derivative_tau = Real(0.001);
    branes::reflex::Pid<Real> pid{cfg};

    const Real dt = Real(1.0 / 8000.0);
    Real y{0};
    const auto start = std::chrono::steady_clock::now();
    for (long i = 0; i < iterations; ++i) {
        const Real setpoint = (i & 0x400) ? Real(1) : Real(-1);
        const Real u = pid.update(setpoint, y, dt);
        y += dt * (u - y);
    }
    const auto stop = std::chrono::steady_clock::now();

    // Keep the result observable.
    if (y != y) {
        std::puts("unexpected NaN");
    }
    const auto ns = std::chrono::duration<double, std::nano>(stop - start).count();
    return ns / static_cast<double>(iterations);
}

}  // namespace

int main(int argc, char** argv) {
    const long iterations = argc > 1 ? std::strtol(argv[1], nullptr, 10) : 10'000'000L;
    if (iterations <= 0) {
        std::fprintf(stderr, "iterations must be positive\n");
        return 1;
    }
    std::printf("Pid<float>::update   %8.2f ns/update\n", ns_per_update<float>(iterations));
    std::printf("Pid<double>::update  %8.2f ns/update\n", ns_per_update<double>(iterations));
    return 0;
}

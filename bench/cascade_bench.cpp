// cascade_bench — per-tick latency of branes::reflex::control::Cascade.
//
// Drives the cascade in position mode with a state that keeps changing (a
// slow orbit with a tumbling attitude) so no loop settles into subnormal
// numbers, and reports the mean time per rate-loop tick for float and double,
// and the mean time per tick of each loop on its own. One cascade tick runs
// the rate loop, an attitude update every 4th tick, and a position update
// every 20th. These are host latency figures (characterization), not a
// timing bound.
//
// Usage: cascade_bench [iterations]   (default 2,000,000)

#include <branes/reflex/control/attitude.hpp>
#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/position.hpp>
#include <branes/reflex/control/rate.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

using namespace branes::reflex;

template <typename T>
control::VehicleState<T> state_at(long i) {
    const T t = T(static_cast<double>(i % 100000) * 1e-3);
    control::VehicleState<T> x;
    x.position = vec3(T(std::cos(double(t))), T(std::sin(double(t))), T(5));
    x.velocity = vec3(T(-std::sin(double(t))), T(std::cos(double(t))), T(0));
    x.attitude = from_rotation_vector(vec3(T(0.2) * T(std::sin(double(t))), T(0.1), t));
    x.body_rates = vec3(T(0.3), T(-0.2), T(1));
    return x;
}

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
    const auto cfg = control::cascade_config_from(model::QuadrotorParams<T>{});
    // Precompute the states so the timing excludes their trigonometry.
    constexpr long kStates = 1024;
    control::VehicleState<T> states[kStates];
    for (long i = 0; i < kStates; ++i) {
        states[i] = state_at<T>(i * 97);
    }
    control::PositionSetpoint<T> sp;
    sp.position = vec3(T(0), T(0), T(5));
    const T dt = T(1e-3);

    control::Cascade<T> cascade(cfg);
    const double tick =
        ns_per_call<T>(iterations, [&](long i) { return cascade.update(states[i % kStates], sp, dt).torque(0); });
    control::RateController<T> rate(cfg.rate);
    const double rate_ns = ns_per_call<T>(iterations, [&](long i) {
        const auto& x = states[i % kStates];
        return rate.update(x.attitude.w > T(0) ? x.body_rates : zero3<T>(), x.body_rates, zero3<T>(), dt)(1);
    });
    const control::AttitudeController<T> attitude(cfg.attitude);
    const double att_ns = ns_per_call<T>(iterations, [&](long i) {
        return attitude.update(states[i % kStates].attitude, states[(i + 7) % kStates].attitude)(0);
    });
    control::PositionController<T> position(cfg.position);
    const double pos_ns = ns_per_call<T>(iterations, [&](long i) {
        const auto& x = states[i % kStates];
        return position.update(x.position, x.velocity, x.attitude, sp, T(0.02)).thrust;
    });
    std::printf("%-6s cascade tick %7.2f ns | rate %7.2f ns | attitude %7.2f ns | position %7.2f ns\n",
                name,
                tick,
                rate_ns,
                att_ns,
                pos_ns);
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

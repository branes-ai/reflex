// Closed-loop harness for the cascade tests: the cascade (designed for the
// nominal vehicle) drives the mixer, a command delay line, and a
// QuadrotorModel (the true vehicle, possibly perturbed) in lockstep at the
// rate-loop period.
#pragma once

#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <array>
#include <cstddef>
#include <vector>

namespace closed_loop {

using namespace branes::reflex;

/// The controller's view of the plant state; `negate` reports −q, the same attitude.
template <typename T>
control::VehicleState<T> observe(const model::QuadrotorState<T>& x, bool negate = false) {
    const Quat<T> q = negate ? Quat<T>{-x.attitude.w, -x.attitude.x, -x.attitude.y, -x.attitude.z} : x.attitude;
    return control::VehicleState<T>{x.position, x.velocity, q, x.body_rates};
}

template <typename T>
class Loop {
public:
    /// @param nominal  the vehicle the cascade and mixer are designed for
    /// @param actual   the vehicle the model simulates
    /// @param delay    rate ticks between the mixer and the rotors (0 = none)
    Loop(const model::QuadrotorParams<T>& nominal,
         const model::QuadrotorParams<T>& actual,
         std::size_t delay = 0,
         T dt = T(1e-3))
        : plant_(actual), mixer_(control::mixer_params_from(nominal)), cascade_(control::cascade_config_from(nominal)),
          dt_(dt), line_(delay + 1) {
        const T w = plant_.hover_rotor_speed();
        for (auto& u : line_) {
            u = model::RotorCommand<T>{{w, w, w, w}};
        }
    }

    Loop(const model::QuadrotorParams<T>& nominal,
         const model::QuadrotorParams<T>& actual,
         const control::CascadeConfig<T>& cfg,
         std::size_t delay = 0,
         T dt = T(1e-3))
        : plant_(actual), mixer_(control::mixer_params_from(nominal)), cascade_(cfg), dt_(dt), line_(delay + 1) {
        const T w = plant_.hover_rotor_speed();
        for (auto& u : line_) {
            u = model::RotorCommand<T>{{w, w, w, w}};
        }
    }

    /// Start airborne at rest at p with the rotors at hover speed.
    void hover_at(const Vec3<T>& p) {
        x_ = model::QuadrotorState<T>{};
        x_.position = p;
        const T w = plant_.hover_rotor_speed();
        x_.rotor_speeds = {w, w, w, w};
    }

    /// One tick; the setpoint is a PositionSetpoint or an AttitudeTarget.
    template <typename Setpoint>
    void step(const Setpoint& sp) {
        const control::Wrench<T> w = cascade_.update(observe(x_, negate_), sp, dt_);
        out_ = mixer_.mix(w);
        cascade_.feedback(out_);
        line_[head_] = mixer_.to_rotor_command(out_.thrusts);
        head_ = (head_ + 1) % line_.size();
        x_ = plant_.step(x_, line_[head_], dt_);  // the oldest command
        t_ += dt_;
    }

    /// Report −q to the controller from now on.
    void negate_attitude(bool on) noexcept {
        negate_ = on;
    }

    [[nodiscard]] const model::QuadrotorState<T>& state() const noexcept {
        return x_;
    }
    model::QuadrotorState<T>& state() noexcept {
        return x_;
    }
    [[nodiscard]] const control::Cascade<T>& cascade() const noexcept {
        return cascade_;
    }
    [[nodiscard]] const control::MixerOutput<T>& mixer_output() const noexcept {
        return out_;
    }
    [[nodiscard]] const model::QuadrotorModel<T>& plant() const noexcept {
        return plant_;
    }
    [[nodiscard]] T time() const noexcept {
        return t_;
    }
    [[nodiscard]] T dt() const noexcept {
        return dt_;
    }

private:
    model::QuadrotorModel<T> plant_;
    control::Mixer<T> mixer_;
    control::Cascade<T> cascade_;
    T dt_;
    std::vector<model::RotorCommand<T>> line_;
    std::size_t head_{0};
    model::QuadrotorState<T> x_{};
    control::MixerOutput<T> out_{};
    T t_{0};
    bool negate_{false};
};

/// Step-response metrics of a scalar response y(t) toward `target` from `start`.
struct StepMetrics {
    double rise90{-1};    ///< first time y crosses 90 % of the step [s]
    double overshoot{0};  ///< peak excursion beyond the target, as a fraction of the step
    double settle{-1};    ///< last time |y − target| > band·|step| [s] (0 if never)
};

inline StepMetrics
step_metrics(const std::vector<double>& t, const std::vector<double>& y, double start, double target, double band) {
    StepMetrics m;
    const double step = target - start;
    const double tol = band * (step < 0 ? -step : step);
    double peak = 0;
    m.settle = 0;
    for (std::size_t i = 0; i < y.size(); ++i) {
        const double progress = (y[i] - start) / step;
        if (m.rise90 < 0 && progress >= 0.9) {
            m.rise90 = t[i];
        }
        peak = progress - 1 > peak ? progress - 1 : peak;
        const double off = y[i] - target;
        if (off > tol || -off > tol) {
            m.settle = t[i];
        }
    }
    m.overshoot = peak;
    return m;
}

}  // namespace closed_loop

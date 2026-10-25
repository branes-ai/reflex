#pragma once
/// @file cascade.hpp
/// @brief The multirotor cascade: position → attitude → body rates → wrench, at three loop rates.
///
///   PositionController ──(q_sp, T)──▶ AttitudeController ──(ω_sp)──▶ RateController ──(τ)──▶ Mixer
///
/// Cascade::update() runs once per rate-loop tick. The attitude loop runs on
/// every attitude_divider-th tick and the position loop on every
/// position_divider-th, each holding its output in between. feedback() takes
/// the mixer's result for the wrench just returned, and the loops' integrators
/// see that saturation on the next tick.
///
/// The interface is plain state in, wrench out, so the same cascade runs in
/// SITL, in hardware-in-the-loop, and on the flight controller.
///
/// Model
/// -----
///   Cascade<T>
///     cfg_        : CascadeConfig<T> [owned, immutable] — the three loops and the dividers
///     position_   : PositionController<T> [owned]
///     attitude_   : AttitudeController<T> [owned]
///     rate_       : RateController<T> [owned]
///     target_     : AttitudeTarget<T> — held output of the position loop (or the caller's target)
///     rate_sp_    : Vec3<T> — held output of the attitude loop
///     wrench_     : Wrench<T> — the last wrench returned
///     saturation_ : Saturation — the mixer's report for wrench_
///     tick_       : unsigned — rate-loop ticks since the last position tick
///
/// Invariants
/// ----------
///   I1: cascade_config_valid(cfg_)                                            — constrains cfg_
///   I2: 0 ≤ tick_ < position_divider                                          — constrains tick_
///   I3: |rate_sp_,i| ≤ max_rate_i; target_.attitude is a unit quaternion      — constrains rate_sp_, target_
///   I4: wrench_.thrust == target_.thrust when wrench_ was last set by update() — constrains wrench_
///
/// Resource contract (BB-S5)
/// -------------------------
/// No allocation, no exceptions, no system calls, no data-dependent loops.
/// A tick is a rate update, plus an attitude update on attitude ticks, plus a
/// position update on position ticks. Each branch inside them (the velocity
/// and force limits, the 180° tilt fallback, the four branches of
/// from_matrix) selects between fixed sequences, so the operation count of
/// each tick kind is bounded by the tick where every limit cuts; a test with
/// an operation-counting scalar checks the bound. Generic in the scalar type.
///
/// Default gains
/// -------------
/// cascade_config_from() designs the loops from a vehicle file by bandwidth
/// separation, from the motor lag τ outwards (ω_n = 1/τ):
///
///   rate      plant α_cmd → ω is 1/(s(τs + 1)). PD on the measurement places
///             the closed loop at ω_n with ζ = 0.7: k_p = τ·ω_n² = 1/τ,
///             k_d = 2ζ·τ·ω_n − 1 = 0.4. The integral zero sits a decade
///             below: k_i = k_p·ω_n/10. Phase margin ≈ 60° nominal.
///   attitude  k = ω_n/5 for roll and pitch, ω_n/10 for yaw. A ratio of 5
///             rather than 4 keeps the attitude overshoot under 25 % with the
///             motors twice as slow, 30 % more inertia, and 4 ms of delay.
///   velocity  k_v = ω_n/14, with an integral zero a decade below (k_i = k_v²/10)
///             and an integral bound of g/2, enough for a 33 % hover-thrust error.
///   position  K_p = ω_n/32, so k_v/K_p ≈ 2.3 and the pair has ζ ≈ 0.76.
///
/// Limits come from the vehicle: angular acceleration from the full
/// differential thrust, collective thrust from 10 % of hover to 80 % of
/// 4·T_max (the rest stays free for torque), tilt 40°. The closed-loop
/// tests in tests/sdk/cascade_closed_loop.cpp hold these gains to the
/// acceptance criteria of reflex#20.

#include <branes/reflex/control/attitude.hpp>
#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/control/pid.hpp>
#include <branes/reflex/control/position.hpp>
#include <branes/reflex/control/rate.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace branes::reflex::control {

/// The vehicle state the cascade needs (an estimator's output, or truth in SITL).
template <typename T>
struct VehicleState {
    Vec3<T> position = zero3<T>();    ///< world [m]
    Vec3<T> velocity = zero3<T>();    ///< world [m/s]
    Quat<T> attitude{};               ///< body → world
    Vec3<T> body_rates = zero3<T>();  ///< body frame [rad/s]
};

template <typename T>
struct CascadeConfig {
    PositionConfig<T> position{};
    AttitudeConfig<T> attitude{};
    RateConfig<T> rate{};
    unsigned attitude_divider{4};   ///< rate ticks per attitude tick (1 kHz / 4 = 250 Hz)
    unsigned position_divider{20};  ///< rate ticks per position tick (1 kHz / 20 = 50 Hz)
};

/// True if `c` satisfies I1: every loop's config valid, attitude_divider ≥ 1,
/// and position_divider a positive multiple of it (the position loop always
/// runs on an attitude tick, so a new target is flown at once).
template <typename T>
[[nodiscard]] bool cascade_config_valid(const CascadeConfig<T>& c) noexcept {
    return position_config_valid(c.position) && attitude_config_valid(c.attitude) && rate_config_valid(c.rate) &&
           c.attitude_divider >= 1 && c.position_divider >= c.attitude_divider &&
           c.position_divider % c.attitude_divider == 0;
}

/// Default gains and limits for vehicle `p` (see the file comment).
/// Pre: model::quadrotor_params_valid(p).
template <typename T>
[[nodiscard]] CascadeConfig<T> cascade_config_from(const model::QuadrotorParams<T>& p) {
    using std::sqrt;
    CascadeConfig<T> c;
    const T tau = p.motor_time_constant;
    const T wn = T(1) / tau;  // rate-loop natural frequency
    const T arm = p.arm_length / sqrt(T(2));
    const std::array<T, 3> torque_max{
        T(2) * arm * p.max_thrust, T(2) * arm * p.max_thrust, T(2) * p.torque_coefficient * p.max_thrust};

    c.rate.inertia = p.inertia;
    for (std::size_t i = 0; i < 3; ++i) {
        const T alpha_max = torque_max[i] / p.inertia[i];
        PidConfig<T>& r = c.rate.axis[i];
        r.kp = T(1) / tau;
        r.kd = T(2) * T(0.7) - T(1);
        r.ki = r.kp * wn / T(10);
        r.output_min = -alpha_max;
        r.output_max = alpha_max;
        r.integral_min = -alpha_max / T(4);
        r.integral_max = alpha_max / T(4);
        r.derivative_tau = tau / T(10);
    }

    const T k_att = wn / T(5);
    c.attitude.gain = {k_att, k_att, k_att / T(2)};  // yaw at ω_n/10
    c.attitude.max_rate = {T(10), T(10), T(4)};

    const T kv = wn / T(14);
    const T g = p.gravity;
    c.position.mass = p.mass;
    c.position.gravity = g;
    const T kp = wn / T(32);
    c.position.position_gain = {kp, kp, kp};
    c.position.max_tilt = T(0.7);  // ≈ 40°
    c.position.min_thrust = T(0.1) * p.mass * g;
    c.position.max_thrust = T(0.8) * T(4) * p.max_thrust;
    c.position.max_speed_xy = T(8);
    c.position.max_speed_up = T(3);
    c.position.max_speed_down = T(2);
    for (std::size_t i = 0; i < 3; ++i) {
        PidConfig<T>& v = c.position.velocity[i];
        v.kp = kv;
        v.ki = kv * kv / T(10);
        v.output_min = -g;
        v.output_max = g;
        v.integral_min = -g / T(2);
        v.integral_max = g / T(2);
    }
    return c;
}

template <typename T>
class Cascade {
public:
    /// Pre: cascade_config_valid(cfg).  Post: I1–I4, every loop reset, tick 0.
    explicit Cascade(const CascadeConfig<T>& cfg)
        : cfg_(cfg), position_(cfg.position), attitude_(cfg.attitude), rate_(cfg.rate) {
        assert(cascade_config_valid(cfg_));  // Pre / I1
        reset();
    }

    /// One rate-loop tick in position mode: the position loop makes the target.
    ///
    /// Pre:  x finite with a unit attitude; sp finite; dt > 0 (dt ≤ 0 returns
    ///       the last wrench and changes nothing).
    /// Post: I1–I4; the returned wrench is {target().thrust, torque}.
    ///
    /// @param dt  the rate-loop period [s]; the outer loops run at multiples of it
    [[nodiscard]] Wrench<T> update(const VehicleState<T>& x, const PositionSetpoint<T>& sp, T dt) {
        if (!(T(0) < dt)) {
            return wrench_;
        }
        if (tick_ == 0) {
            target_ = position_.update(
                x.position, x.velocity, x.attitude, sp, dt * T(cfg_.position_divider), saturation_.thrust);
        }
        return inner(x, dt);
    }

    /// One rate-loop tick in attitude mode: the caller supplies the target and
    /// the position loop does not run.
    ///
    /// Pre:  as the position-mode update(); target.attitude unit, target.thrust finite.
    /// Post: as the position-mode update().
    [[nodiscard]] Wrench<T> update(const VehicleState<T>& x, const AttitudeTarget<T>& target, T dt) {
        if (!(T(0) < dt)) {
            return wrench_;
        }
        target_ = target;
        return inner(x, dt);
    }

    /// The mixer's result for the wrench update() just returned.
    /// Pre: out is mix(wrench()).  Post: the next tick's integrators see its saturation.
    void feedback(const MixerOutput<T>& out) noexcept {
        saturation_ = saturation_from(wrench_, out);
    }

    /// Pre: none.  Post: as after construction.
    void reset() noexcept {
        position_.reset();
        rate_.reset();
        target_ = position_.target();
        rate_sp_ = zero3<T>();
        wrench_ = Wrench<T>{target_.thrust, zero3<T>()};
        saturation_ = Saturation{};
        tick_ = 0;
        assert(check_invariant());
    }

    [[nodiscard]] const CascadeConfig<T>& config() const noexcept {
        return cfg_;
    }
    [[nodiscard]] const PositionController<T>& position() const noexcept {
        return position_;
    }
    [[nodiscard]] const AttitudeController<T>& attitude() const noexcept {
        return attitude_;
    }
    [[nodiscard]] const RateController<T>& rate() const noexcept {
        return rate_;
    }
    [[nodiscard]] const AttitudeTarget<T>& target() const noexcept {
        return target_;
    }
    [[nodiscard]] const Vec3<T>& rate_setpoint() const noexcept {
        return rate_sp_;
    }
    [[nodiscard]] const Wrench<T>& wrench() const noexcept {
        return wrench_;
    }
    [[nodiscard]] const Saturation& saturation() const noexcept {
        return saturation_;
    }

private:
    /// The attitude and rate loops, then the tick advance.
    [[nodiscard]] Wrench<T> inner(const VehicleState<T>& x, T dt) {
        if (tick_ % cfg_.attitude_divider == 0) {
            rate_sp_ = attitude_.update(x.attitude, target_.attitude);
        }
        const Vec3<T> torque = rate_.update(rate_sp_, x.body_rates, zero3<T>(), dt, saturation_.torque);
        wrench_ = Wrench<T>{target_.thrust, torque};
        tick_ = (tick_ + 1) % cfg_.position_divider;  // I2
        assert(check_invariant());
        return wrench_;
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        bool ok = cascade_config_valid(cfg_) && tick_ < cfg_.position_divider;  // I1, I2
        for (std::size_t i = 0; i < 3; ++i) {
            const T m = cfg_.attitude.max_rate[i];
            ok = ok && !(rate_sp_(i) < -m) && !(m < rate_sp_(i));  // I3
        }
        return ok && wrench_.thrust == target_.thrust;  // I4
    }

    CascadeConfig<T> cfg_;
    PositionController<T> position_;
    AttitudeController<T> attitude_;
    RateController<T> rate_;
    AttitudeTarget<T> target_{};
    Vec3<T> rate_sp_ = zero3<T>();
    Wrench<T> wrench_{};
    Saturation saturation_{};
    unsigned tick_{0};
};

}  // namespace branes::reflex::control

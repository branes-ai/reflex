#pragma once
/// @file rate.hpp
/// @brief Body-rate loop: body-rate setpoint to body torque, one PID per axis.
///
/// Law
/// ---
///   α_sp,i = clamp(PID_i(ω_sp,i, ω_i) + α_ff,i, α_min,i, α_max,i)     i = roll, pitch, yaw
///   τ      = J·α_sp + ω × Jω
///
/// The PIDs output angular acceleration [rad/s²], not torque, so their gains
/// are in 1/s and 1/s² and one loop shape carries across vehicles of
/// different size. J scales the result to torque, and ω × Jω cancels the
/// gyroscopic coupling the plant adds (model::QuadrotorModel: ω̇ = J⁻¹(τ − ω × Jω)).
/// Anti-windup is the PID's own (conditional integration on its output
/// limits), plus the mixer's saturation report from the previous tick.
///
/// Model
/// -----
///   RateController<T>
///     cfg_    : RateConfig<T> [owned, immutable] — inertia, per-axis PID configs
///     pid_    : std::array<Pid<T>, 3> [owned] — roll, pitch, yaw
///     torque_ : Vec3<T> — the last torque returned
///
/// Invariants
/// ----------
///   I1: rate_config_valid(cfg_)                                         — constrains cfg_
///   I2: each pid_[i] was built from cfg_.axis[i] (so Pid's I1–I4 hold)  — constrains pid_
///   I3: torque_ is finite                                               — constrains torque_
///
/// Body frame x forward, y left, z up, as in the model. Constant time, no
/// allocation, no exceptions; generic in the scalar type.

#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/control/pid.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace branes::reflex::control {

/// Rate-loop configuration. Each axis PID's output limits are the angular
/// acceleration the loop may command [rad/s²].
template <typename T>
struct RateConfig {
    std::array<T, 3> inertia{T(0.003), T(0.003), T(0.005)};  ///< diagonal J [kg·m²]
    std::array<PidConfig<T>, 3> axis{};                      ///< roll, pitch, yaw
};

/// True if `c` satisfies I1: J finite and > 0, every axis a valid PidConfig.
template <typename T>
[[nodiscard]] bool rate_config_valid(const RateConfig<T>& c) noexcept {
    using std::isfinite;
    bool ok = true;
    for (std::size_t i = 0; i < 3; ++i) {
        ok = ok && isfinite(c.inertia[i]) && c.inertia[i] > T(0) && pid_config_valid(c.axis[i]);
    }
    return ok;
}

template <typename T>
class RateController {
public:
    /// Pre: rate_config_valid(cfg).  Post: I1–I3; every PID in its reset state; torque 0.
    explicit RateController(const RateConfig<T>& cfg)
        : cfg_(cfg), pid_{Pid<T>(cfg.axis[0]), Pid<T>(cfg.axis[1]), Pid<T>(cfg.axis[2])} {
        assert(rate_config_valid(cfg_));  // Pre / I1
        assert(check_invariant());
    }

    /// One rate-loop tick.
    ///
    /// Pre:  every argument finite. A non-positive dt is a timing fault in the
    ///       caller: the last torque is returned and no state changes.
    /// Post: returns τ = J·α + ω × Jω with α_i ∈ [α_min,i, α_max,i]; an axis
    ///       whose saturation[i] is nonzero integrates only out of that
    ///       saturation; I1–I3.
    ///
    /// @param rate_sp     body-rate setpoint ω_sp [rad/s]
    /// @param rate        measured body rates ω [rad/s]
    /// @param accel_ff    angular-acceleration feed-forward α_ff [rad/s²]
    /// @param dt          time since the previous tick [s]
    /// @param saturation  the mixer's report for the previous tick's torque
    [[nodiscard]] Vec3<T> update(const Vec3<T>& rate_sp,
                                 const Vec3<T>& rate,
                                 const Vec3<T>& accel_ff,
                                 T dt,
                                 const std::array<int, 3>& saturation = {}) {
        if (!(T(0) < dt)) {
            return torque_;  // Pre violated: hold (I1–I3 untouched)
        }
        Vec3<T> alpha = zero3<T>();
        for (std::size_t i = 0; i < 3; ++i) {
            const PidConfig<T>& c = cfg_.axis[i];
            const T u = pid_[i].update(rate_sp(i), rate(i), dt, saturation[i]) + accel_ff(i);
            alpha(i) = u < c.output_min ? c.output_min : (c.output_max < u ? c.output_max : u);
        }
        const Vec3<T> j_omega = vec3(cfg_.inertia[0] * rate(0), cfg_.inertia[1] * rate(1), cfg_.inertia[2] * rate(2));
        torque_ = add(vec3(cfg_.inertia[0] * alpha(0), cfg_.inertia[1] * alpha(1), cfg_.inertia[2] * alpha(2)),
                      cross(rate, j_omega));
        assert(check_invariant());
        return torque_;
    }

    /// Pre: none.  Post: every PID reset, torque 0, I1–I3.
    void reset() noexcept {
        for (Pid<T>& p : pid_) {
            p.reset();
        }
        torque_ = zero3<T>();
        assert(check_invariant());
    }

    [[nodiscard]] const RateConfig<T>& config() const noexcept {
        return cfg_;
    }

    /// The PID of axis i (0 roll, 1 pitch, 2 yaw), for its integral and rate.
    [[nodiscard]] const Pid<T>& axis(std::size_t i) const noexcept {
        return pid_[i];
    }

    /// The last torque update() returned.
    [[nodiscard]] const Vec3<T>& torque() const noexcept {
        return torque_;
    }

private:
    [[nodiscard]] bool check_invariant() const noexcept {
        using std::isfinite;
        return rate_config_valid(cfg_)                                                   // I1
               && isfinite(torque_(0)) && isfinite(torque_(1)) && isfinite(torque_(2));  // I3
    }

    RateConfig<T> cfg_;
    std::array<Pid<T>, 3> pid_;
    Vec3<T> torque_ = zero3<T>();
};

}  // namespace branes::reflex::control

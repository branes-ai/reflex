#pragma once
/// @file pid.hpp
/// @brief Discrete-time PID controller for flight-control inner loops.
///
/// The controller is the textbook parallel form
///
///   u = Kp * e + Ki * integral(e dt) - Kd * d(y)/dt,   e = r - y
///
/// with the refinements a real flight controller needs:
///
///  - **Derivative on measurement.** The D term acts on -dy/dt rather than
///    de/dt, so a step in the setpoint does not produce a derivative kick.
///  - **Derivative low-pass filter.** A first-order filter with time constant
///    `derivative_tau` attenuates sensor noise that differentiation amplifies.
///  - **Integral clamping.** The integral contribution is bounded to
///    [integral_min, integral_max] (in output units).
///  - **Conditional integration (anti-windup).** While the output is
///    saturated, the integral is frozen if integrating would drive the output
///    further into saturation; it still integrates when the error would pull
///    the output back out.
///  - **Output saturation** to [output_min, output_max].
///
/// Model
/// -----
///   Pid<Real>
///     cfg_              : PidConfig<Real> [owned, immutable] — gains and limits
///     integral_         : Real — integral contribution Ki * integral(e dt), output units
///     rate_             : Real — filtered measurement rate dy/dt
///     prev_measurement_ : Real — y from the previous update
///     output_           : Real — last commanded output
///     primed_           : bool — whether prev_measurement_ holds a real sample
///
/// Invariants
/// ----------
///   I1: cfg_ is valid (pid_config_valid): gains >= 0, min <= max for both
///       limit pairs, derivative_tau >= 0                   — constrains cfg_
///   I2: integral_min <= integral_ <= integral_max          — constrains integral_
///   I3: output_min <= output_ <= output_max                — constrains output_
///   I4: !primed_ implies rate_ == 0                        — constrains rate_, primed_
///
/// Gains and limits are fixed at construction. Per the platform's lifecycle
/// rule there is no dynamic reconfiguration: to change gains, construct a new
/// controller (Unconfigured -> Inactive -> Active), never mutate one in the
/// hot path.
///
/// The controller is templated on the scalar type and uses only +, -, *, /
/// and <, so it runs in double, float, or custom arithmetic (e.g. Universal
/// posits). It never allocates, never throws, and has a constant-time update.

#include <algorithm>
#include <cassert>

namespace branes::reflex {

/// Gains and limits for a Pid controller. Plain aggregate, passed by value.
///
/// The defaults (unity output range, integral range equal to the output range,
/// no gains) describe a normalized actuator command; callers set the gains and
/// any limits their loop needs.
template <typename Real>
struct PidConfig {
    Real kp{0};  ///< proportional gain [output / error]
    Real ki{0};  ///< integral gain [output / (error * s)]
    Real kd{0};  ///< derivative gain [output * s / error]

    Real output_min{-1};  ///< lower output saturation
    Real output_max{1};   ///< upper output saturation

    Real integral_min{-1};  ///< lower bound of the integral contribution (output units)
    Real integral_max{1};   ///< upper bound of the integral contribution (output units)

    /// Time constant [s] of the first-order low-pass filter on the measurement
    /// rate. 0 disables the filter (raw finite difference).
    Real derivative_tau{0};
};

/// True if `cfg` satisfies invariant I1 of Pid.
template <typename Real>
[[nodiscard]] constexpr bool pid_config_valid(const PidConfig<Real>& cfg) noexcept {
    const Real zero{0};
    return !(cfg.kp < zero) && !(cfg.ki < zero) && !(cfg.kd < zero) && !(cfg.output_max < cfg.output_min) &&
           !(cfg.integral_max < cfg.integral_min) && !(cfg.derivative_tau < zero);
}

/// Discrete-time PID controller with derivative-on-measurement, derivative
/// filtering, integral clamping, conditional-integration anti-windup, and
/// output saturation. See the file comment for the model and invariants.
template <typename Real>
class Pid {
public:
    /// Pre:  pid_config_valid(cfg).
    /// Post: controller is in the reset state (integral 0, output 0 clamped
    ///       into the output range, unprimed). Establishes I1-I4.
    explicit constexpr Pid(const PidConfig<Real>& cfg) noexcept : cfg_{cfg} {
        assert(pid_config_valid(cfg_));  // Pre / I1
        reset();
    }

    /// Advance the controller by one sample.
    ///
    /// Pre:  none on the arguments. A non-positive `dt` is a fault in the
    ///       caller's timing; the controller holds its last output and leaves
    ///       its state unchanged rather than dividing by zero.
    /// Post: returns the new output u, with output_min <= u <= output_max
    ///       (I3); the integral stays within its bounds (I2); the controller is
    ///       primed (I4 trivially holds).
    ///
    /// @param setpoint     reference r
    /// @param measurement  measured process value y
    /// @param dt           time since the previous update [s], > 0
    /// @return the saturated control output
    constexpr Real update(Real setpoint, Real measurement, Real dt) noexcept {
        const Real zero{0};
        if (!(zero < dt)) {
            return output_;  // Pre violated: hold output, keep state (I1-I4 untouched)
        }

        const Real error = setpoint - measurement;

        // D term on the measurement, so setpoint steps do not kick. The first
        // sample has no history: treat its rate as zero.
        const Real raw_rate = primed_ ? (measurement - prev_measurement_) / dt : zero;
        if (cfg_.derivative_tau > zero) {
            rate_ = (cfg_.derivative_tau * rate_ + dt * raw_rate) / (cfg_.derivative_tau + dt);
        } else {
            rate_ = raw_rate;
        }
        prev_measurement_ = measurement;
        primed_ = true;

        const Real p_term = cfg_.kp * error;
        const Real d_term = -(cfg_.kd * rate_);

        // Integrate, clamped to the integral bounds (I2).
        const Real candidate = std::clamp(integral_ + cfg_.ki * error * dt, cfg_.integral_min, cfg_.integral_max);

        // Conditional integration: if the output would saturate and the error
        // pushes further into that saturation, freeze the integral.
        const Real unsaturated = p_term + candidate + d_term;
        const bool winding_up_high = cfg_.output_max < unsaturated && zero < error;
        const bool winding_up_low = unsaturated < cfg_.output_min && error < zero;
        if (!winding_up_high && !winding_up_low) {
            integral_ = candidate;
        }

        output_ = std::clamp(p_term + integral_ + d_term, cfg_.output_min, cfg_.output_max);  // I3

        assert(check_invariant());
        return output_;
    }

    /// Return to the initial state: zero integral and rate, unprimed, output
    /// set to 0 clamped into the output range.
    ///
    /// Pre:  none.  Post: same state as a freshly constructed controller.
    constexpr void reset() noexcept {
        const Real zero{0};
        integral_ = std::clamp(zero, cfg_.integral_min, cfg_.integral_max);  // I2
        rate_ = zero;                                                        // I4
        prev_measurement_ = zero;
        output_ = std::clamp(zero, cfg_.output_min, cfg_.output_max);  // I3
        primed_ = false;
        assert(check_invariant());
    }

    /// The configuration this controller was built with.
    [[nodiscard]] constexpr const PidConfig<Real>& config() const noexcept {
        return cfg_;
    }

    /// The current integral contribution Ki * integral(e dt), in output units.
    [[nodiscard]] constexpr Real integral() const noexcept {
        return integral_;
    }

    /// The filtered measurement rate dy/dt used by the D term.
    [[nodiscard]] constexpr Real measurement_rate() const noexcept {
        return rate_;
    }

    /// The most recent output (0 clamped into range before the first update).
    [[nodiscard]] constexpr Real output() const noexcept {
        return output_;
    }

private:
    [[nodiscard]] constexpr bool check_invariant() const noexcept {
        return pid_config_valid(cfg_)                                                   // I1
               && !(integral_ < cfg_.integral_min) && !(cfg_.integral_max < integral_)  // I2
               && !(output_ < cfg_.output_min) && !(cfg_.output_max < output_)          // I3
               && (primed_ || !(rate_ < Real{0} || Real{0} < rate_));                   // I4
    }

    PidConfig<Real> cfg_;
    Real integral_{0};
    Real rate_{0};
    Real prev_measurement_{0};
    Real output_{0};
    bool primed_{false};
};

}  // namespace branes::reflex

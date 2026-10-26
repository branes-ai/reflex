#pragma once
/// @file mixer.hpp
/// @brief Quad X mixer: collective thrust and body torques to four rotor thrusts, with saturation priority.
///
/// Conventions
/// -----------
/// Those of model::QuadrotorModel: body frame x forward, y left, z up; PX4
/// Quad X rotor order 0 front-right CCW, 1 rear-left CCW, 2 front-left CW,
/// 3 rear-right CW. Rotors sit at (±a, ±a, 0) with a = L/√2.
///
/// Allocation
/// ----------
/// The wrench w = (F, τx, τy, τz) is B·T for the rotor thrusts T:
///
///   F  =        T0 + T1 + T2 + T3
///   τx = a   · (−T0 + T1 + T2 − T3)
///   τy = a   · (−T0 + T1 − T2 + T3)
///   τz = k_m · (−T0 − T1 + T2 + T3)
///
/// The rows of B are orthogonal, so B⁻¹ = Bᵀ · diag(1/4, 1/(4a²), 1/(4a²), 1/(4k_m²)):
///
///   T_i = F/4 + sx_i·τx/(4a) + sy_i·τy/(4a) + sz_i·τz/(4k_m)
///
/// Saturation priority
/// -------------------
/// Rotors 0 and 1 (pair a) differ only by the roll/pitch term ±d_a, and so do
/// rotors 2 and 3 (pair b, ±d_b); yaw moves pair a by −Y and pair b by +Y.
/// So every allocation is T = (u_a ± σd_a, u_b ± σd_b) with pair means
/// u_a = c − t and u_b = c + t, collective c and yaw offset t. mix() picks,
/// lexicographically:
///
///   1. roll/pitch: the largest σ ∈ (0, 1] with 2σ·max(|d_a|, |d_b|) ≤ T_max − T_min;
///   2. collective: the c closest to F/4 for which some yaw t = s·Y, s ∈ [0, 1], fits;
///   3. yaw: the largest s ∈ [0, 1] that fits around that c.
///
/// Yaw gives way first, then collective thrust; roll and pitch shrink only when
/// they cannot fit at any thrust. The collective moves both ways: at low
/// throttle it rises to keep roll and pitch authority (airmode), so arming and
/// motor idle belong to the state machine around the mixer, not to the mixer.
/// Each step is a clamp of a continuous bound, so the output is continuous in
/// the demand, and a demand that already fits comes back unchanged.
///
/// Model
/// -----
///   Mixer<T>
///     p_      : MixerParams<T> [owned, immutable] — geometry, coefficients, thrust limits
///     c_xy_   : T [derived] — 1/(4a), roll/pitch torque to per-rotor thrust
///     c_z_    : T [derived] — 1/(4k_m), yaw torque to per-rotor thrust
///     a_      : T [derived] — L/√2, the moment arm about x and y
///
/// Invariants
/// ----------
///   I1: mixer_params_valid(p_)                                   — constrains p_
///   I2: a_ = L/√2, c_xy_ = 1/(4a_), c_z_ = 1/(4k_m)               — constrains a_, c_xy_, c_z_
///
/// The constructor establishes both and no operation mutates the mixer, so they
/// hold for its lifetime. Every operation is a pure, constant-time function that
/// never allocates and never throws.

#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace branes::reflex::control {

inline constexpr std::size_t kMixerRotors = model::kRotors;

/// A body-frame wrench: collective thrust along +z_b [N] and torque [N·m].
template <typename T>
struct Wrench {
    T thrust{0};
    Vec3<T> torque = zero3<T>();
};

/// Mixer parameters, per rotor. Plain aggregate, passed by value.
template <typename T>
struct MixerParams {
    T arm_length{T(0.11)};            ///< L: center to rotor axis [m]
    T torque_coefficient{T(0.016)};   ///< k_m: yaw torque = k_m·T [m]
    T thrust_coefficient{T(8.9e-7)};  ///< k_f: T = k_f·ω² [N / (rad/s)²]
    T min_thrust{0};                  ///< T_min per rotor [N]
    T max_thrust{12};                 ///< T_max per rotor [N]
};

/// True if `p` satisfies I1: every value finite, L > 0, k_m > 0, k_f > 0, 0 ≤ T_min < T_max.
template <typename T>
[[nodiscard]] bool mixer_params_valid(const MixerParams<T>& p) noexcept {
    using std::isfinite;
    const T zero{0};
    return isfinite(p.arm_length) && isfinite(p.torque_coefficient) && isfinite(p.thrust_coefficient) &&
           isfinite(p.min_thrust) && isfinite(p.max_thrust) && p.arm_length > zero && p.torque_coefficient > zero &&
           p.thrust_coefficient > zero && p.min_thrust >= zero && p.min_thrust < p.max_thrust;
}

/// The mixer for the vehicle the model simulates, with T_min = 0.
template <typename T>
[[nodiscard]] MixerParams<T> mixer_params_from(const model::QuadrotorParams<T>& q) noexcept {
    return MixerParams<T>{q.arm_length, q.torque_coefficient, q.thrust_coefficient, T(0), q.max_thrust};
}

/// The result of mix(): rotor thrusts within the limits, the wrench they make,
/// and how much of the demand was given up.
template <typename T>
struct MixerOutput {
    std::array<T, kMixerRotors> thrusts{};  ///< per rotor [N], in [T_min, T_max]
    Wrench<T> achieved{};                   ///< wrench(thrusts)
    T rp_scale{1};                          ///< σ: achieved roll/pitch torque = σ · demanded
    T yaw_scale{1};                         ///< s: achieved yaw torque = s · demanded
    bool thrust_shifted{false};             ///< the collective moved off F/4 to keep roll/pitch
};

/// Which way mix() cut each channel of a demand: +1 it delivered less than
/// the demand (cut high), −1 more (cut low), 0 the channel was met. The
/// control loops pass it to their integrators (Pid's external_saturation) on
/// the next tick, so they do not wind up against the mixer.
struct Saturation {
    std::array<int, 3> torque{};  ///< roll, pitch, yaw
    int thrust{0};                ///< collective
};

/// The saturation report for demand `w` and its mix() result `out`.
/// σ < 1 shrinks roll and pitch toward zero, and s < 1 shrinks yaw, so each
/// cut is in the direction of the demand's sign; a shifted collective is cut
/// in the direction of demand − achieved.
template <typename T>
[[nodiscard]] Saturation saturation_from(const Wrench<T>& w, const MixerOutput<T>& out) noexcept {
    const auto sign = [](T v) { return v > T(0) ? 1 : (v < T(0) ? -1 : 0); };
    Saturation s;
    if (out.rp_scale < T(1)) {
        s.torque[0] = sign(w.torque(0));
        s.torque[1] = sign(w.torque(1));
    }
    if (out.yaw_scale < T(1)) {
        s.torque[2] = sign(w.torque(2));
    }
    if (out.thrust_shifted) {
        s.thrust = sign(w.thrust - out.achieved.thrust);
    }
    return s;
}

template <typename T>
class Mixer {
public:
    /// Pre: mixer_params_valid(p) (I1).  Post: I1, I2.
    explicit Mixer(const MixerParams<T>& p)
        : p_(p), a_(arm(p)), c_xy_(T(1) / (T(4) * a_)), c_z_(T(1) / (T(4) * p.torque_coefficient)) {
        assert(mixer_params_valid(p_));  // Pre / I1
    }

    [[nodiscard]] const MixerParams<T>& params() const noexcept {
        return p_;
    }

    /// B⁻¹·w with no limits. Pre: w finite.  Post: wrench(allocate(w)) == w (to rounding).
    [[nodiscard]] std::array<T, kMixerRotors> allocate(const Wrench<T>& w) const {
        assert(finite(w));  // Pre
        const T c = w.thrust / T(4);
        const T da = roll_pitch_a(w);
        const T db = roll_pitch_b(w);
        const T y = yaw_offset(w);
        return {c - y + da, c - y - da, c + y + db, c + y - db};
    }

    /// B·t, the wrench the rotor thrusts make. Pre: t finite.
    [[nodiscard]] Wrench<T> wrench(const std::array<T, kMixerRotors>& t) const {
        Wrench<T> w;
        w.thrust = t[0] + t[1] + t[2] + t[3];
        w.torque = vec3(a_ * (-t[0] + t[1] + t[2] - t[3]),
                        a_ * (-t[0] + t[1] - t[2] + t[3]),
                        p_.torque_coefficient * (-t[0] - t[1] + t[2] + t[3]));
        return w;
    }

    /// Rotor thrusts for w within [T_min, T_max], by the priority in the file comment.
    /// Pre:  w finite.
    /// Post: T_min ≤ thrusts_i ≤ T_max; achieved == wrench(thrusts);
    ///       achieved.torque = (σ·τx, σ·τy, s·τz) with σ = rp_scale ∈ (0, 1], s = yaw_scale ∈ [0, 1];
    ///       if allocate(w) is within the limits, thrusts == allocate(w), σ = s = 1, !thrust_shifted.
    [[nodiscard]] MixerOutput<T> mix(const Wrench<T>& w) const {
        assert(finite(w));  // Pre
        using std::abs;
        const T range = p_.max_thrust - p_.min_thrust;
        const T c0 = w.thrust / T(4);
        T da = roll_pitch_a(w);
        T db = roll_pitch_b(w);
        const T y = yaw_offset(w);

        // 1. Roll/pitch: each pair spans 2|d|, which must fit the thrust range.
        const T span = T(2) * max_of(abs(da), abs(db));
        const T sigma = span > range ? range / span : T(1);
        da = sigma * da;
        db = sigma * db;

        // Pair-mean intervals that keep both rotors of a pair in limits.
        const T lo_a = p_.min_thrust + abs(da);
        const T hi_a = p_.max_thrust - abs(da);
        const T lo_b = p_.min_thrust + abs(db);
        const T hi_b = p_.max_thrust - abs(db);
        // The yaw offset t ranges over [0, Y] (or [Y, 0]).
        const T t_min = min_of(T(0), y);
        const T t_max = max_of(T(0), y);

        // 2. Collective: the projection onto c of { lo_a ≤ c − t ≤ hi_a, lo_b ≤ c + t ≤ hi_b, t_min ≤ t ≤ t_max }.
        const T c_min = max_of(max_of((lo_a + lo_b) / T(2), lo_b - t_max), lo_a + t_min);
        const T c_max = min_of(min_of((hi_a + hi_b) / T(2), hi_a + t_max), hi_b - t_min);
        const T c = clamp_to(c0, c_min, c_max);

        // 3. Yaw: the t closest to Y that fits around c.
        const T t_lo = max_of(max_of(c - hi_a, lo_b - c), t_min);
        const T t_hi = min_of(min_of(c - lo_a, hi_b - c), t_max);
        const T t = clamp_to(y, t_lo, t_hi);

        MixerOutput<T> out;
        const T u_a = c - t;
        const T u_b = c + t;
        out.thrusts = {u_a + da, u_a - da, u_b + db, u_b - db};
        for (T& ti : out.thrusts) {
            ti = clamp_to(ti, p_.min_thrust, p_.max_thrust);  // Post: bounds, against rounding at the edges
        }
        out.achieved = wrench(out.thrusts);
        out.rp_scale = sigma;
        out.yaw_scale = y == T(0) ? T(1) : clamp_to(t / y, T(0), T(1));
        out.thrust_shifted = c != c0;
        return out;
    }

    /// Rotor speed commands with k_f·ω_i² = t_i (full battery; the loops absorb the sag).
    /// Pre: every t_i finite and ≥ 0.
    [[nodiscard]] model::RotorCommand<T> to_rotor_command(const std::array<T, kMixerRotors>& t) const {
        using std::sqrt;
        model::RotorCommand<T> u;
        for (std::size_t i = 0; i < kMixerRotors; ++i) {
            using std::isfinite;
            assert(isfinite(t[i]) && t[i] >= T(0));  // Pre
            u.speeds[i] = sqrt(t[i] / p_.thrust_coefficient);
        }
        return u;
    }

private:
    [[nodiscard]] static T arm(const MixerParams<T>& p) {
        using std::sqrt;
        return p.arm_length / sqrt(T(2));
    }

    [[nodiscard]] static bool finite(const Wrench<T>& w) noexcept {
        using std::isfinite;
        return isfinite(w.thrust) && isfinite(w.torque(0)) && isfinite(w.torque(1)) && isfinite(w.torque(2));
    }

    /// Roll/pitch term of rotor 0 (rotor 1 takes its negative).
    [[nodiscard]] T roll_pitch_a(const Wrench<T>& w) const {
        return -c_xy_ * (w.torque(0) + w.torque(1));
    }

    /// Roll/pitch term of rotor 2 (rotor 3 takes its negative).
    [[nodiscard]] T roll_pitch_b(const Wrench<T>& w) const {
        return c_xy_ * (w.torque(0) - w.torque(1));
    }

    /// Yaw term Y: pair b (rotors 2, 3) gets +Y, pair a (rotors 0, 1) gets −Y.
    [[nodiscard]] T yaw_offset(const Wrench<T>& w) const {
        return c_z_ * w.torque(2);
    }

    [[nodiscard]] static T min_of(T x, T y) noexcept {
        return y < x ? y : x;
    }

    [[nodiscard]] static T max_of(T x, T y) noexcept {
        return x < y ? y : x;
    }

    /// x clamped to [lo, hi]. When rounding leaves hi < lo, the midpoint.
    [[nodiscard]] static T clamp_to(T x, T lo, T hi) noexcept {
        if (hi < lo) {
            return (lo + hi) / T(2);
        }
        return x < lo ? lo : (hi < x ? hi : x);
    }

    MixerParams<T> p_;
    T a_;
    T c_xy_;
    T c_z_;
};

}  // namespace branes::reflex::control

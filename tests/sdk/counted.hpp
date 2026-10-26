// Counted: a double that counts its arithmetic, for the BB-S5 operation-count
// tests. +, −, ×, ÷ (including compound forms) count as ops; sqrt, sin, cos
// count as calls. Negation, comparison, abs, and isfinite are not counted:
// they are not arithmetic in the sense of BB-S5.
#pragma once

#include <cmath>
#include <compare>
#include <cstdint>
#include <limits>

namespace counted {

struct Counted {
    static inline std::uint64_t ops = 0;
    static inline std::uint64_t calls = 0;

    static void reset() noexcept {
        ops = 0;
        calls = 0;
    }

    double v{0};

    constexpr Counted() = default;
    constexpr Counted(double x) : v(x) {}  // NOLINT(google-explicit-constructor): a scalar stand-in

    friend Counted operator+(Counted a, Counted b) {
        ++ops;
        return a.v + b.v;
    }
    friend Counted operator-(Counted a, Counted b) {
        ++ops;
        return a.v - b.v;
    }
    friend Counted operator*(Counted a, Counted b) {
        ++ops;
        return a.v * b.v;
    }
    friend Counted operator/(Counted a, Counted b) {
        ++ops;
        return a.v / b.v;
    }
    friend Counted operator-(Counted a) {
        return -a.v;
    }
    Counted& operator+=(Counted b) {
        return *this = *this + b;
    }
    Counted& operator-=(Counted b) {
        return *this = *this - b;
    }
    Counted& operator*=(Counted b) {
        return *this = *this * b;
    }
    Counted& operator/=(Counted b) {
        return *this = *this / b;
    }
    friend constexpr std::partial_ordering operator<=>(Counted a, Counted b) {
        return a.v <=> b.v;
    }
    friend constexpr bool operator==(Counted a, Counted b) {
        return a.v == b.v;
    }
    friend Counted sqrt(Counted a) {
        ++calls;
        return std::sqrt(a.v);
    }
    friend Counted sin(Counted a) {
        ++calls;
        return std::sin(a.v);
    }
    friend Counted cos(Counted a) {
        ++calls;
        return std::cos(a.v);
    }
    friend Counted abs(Counted a) {
        return std::abs(a.v);
    }
    friend bool isfinite(Counted a) {
        return std::isfinite(a.v);
    }
};

}  // namespace counted

template <>
struct std::numeric_limits<counted::Counted> : std::numeric_limits<double> {
    static constexpr counted::Counted epsilon() noexcept {
        return std::numeric_limits<double>::epsilon();
    }
    static constexpr counted::Counted min() noexcept {
        return std::numeric_limits<double>::min();
    }
};

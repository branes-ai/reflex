#pragma once

/// reflex-trace/1 writer: the C++ side of the trace format that the docs-site
/// player renders (docs-site/src/content/docs/standards/animation.md, "Trace
/// schema"). Host-side tooling only; nothing in branes::reflex depends on it.
///
/// A Trace collects samples in memory and serializes them once, so the file is
/// either complete and valid or not written at all.
///
/// Invariants (checked on every mutation, so a Trace is always serializable):
///   W1: t is strictly increasing, starts at 0, and when dt_trace > 0 each
///       t[k] equals k·dt_trace within 1e-9·dt_trace.
///   W2: every channel holds exactly one entry per element of t, after each
///       sample() call completes.
///   W3: every entry of a channel has the channel's fixed width; every value
///       is finite.
///   W4: a reserved channel has its reserved width; any other channel names a
///       unit.
///   W5: event times lie within [t.front(), t.back()] at serialization.
///   W6: a check's comparison is "<=" or ">=", and its value and limit are
///       finite.
/// Violations throw std::invalid_argument; tools may use exceptions.

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace branes::reflex::trace {

inline constexpr std::string_view kSchema = "reflex-trace/1";

/// What the trace is (ANI-2). commit and compiler are filled in by
/// Provenance::detect() when left empty.
struct Provenance {
    std::string commit;    ///< short git hash; from REFLEX_TRACE_COMMIT if empty
    std::string block;     ///< e.g. "model/quadrotor"
    std::string scenario;  ///< repo-relative path of the scenario source
    long seed{0};
    std::string compiler;  ///< from the predefined macros if empty
    double dt_sim{0};      ///< integration step [s]
    double dt_trace{0};    ///< sample spacing [s]; 0 means non-uniform
};

/// A threshold check computed by the harness, never by the player (ANI-4).
struct Check {
    std::string id;      ///< requirement ID, e.g. "BB-V3"
    std::string metric;  ///< e.g. "attitude_norm_error_max"
    std::string unit;    ///< "1" for dimensionless
    double value{0};
    double limit{0};
    std::string comparison;  ///< "<=" or ">="
    [[nodiscard]] bool pass() const {
        return comparison == "<=" ? value <= limit : value >= limit;
    }
};

/// Width of a reserved channel, or 0 if `name` is not reserved. Setpoints of a
/// reserved quantity take that quantity's width; roll/pitch/yaw setpoints are
/// scalars in rad.
[[nodiscard]] inline std::size_t reserved_width(std::string_view name) {
    static const std::map<std::string_view, std::size_t> widths{
        {"position", 3},
        {"velocity", 3},
        {"attitude", 4},
        {"body_rates", 3},
        {"rotor_speeds", 4},
        {"rotor_thrusts", 4},
        {"wrench", 4},
    };
    if (const auto it = widths.find(name); it != widths.end()) {
        return it->second;
    }
    constexpr std::string_view suffix = "_setpoint";
    if (name.size() > suffix.size() && name.substr(name.size() - suffix.size()) == suffix) {
        const std::string_view base = name.substr(0, name.size() - suffix.size());
        if (base == "roll" || base == "pitch" || base == "yaw") {
            return 1;
        }
        if (const auto it = widths.find(base); it != widths.end()) {
            return it->second;
        }
    }
    return 0;
}

/// The value of an environment variable, or "" if unset. MSVC deprecates
/// std::getenv (C4996), so use its _dupenv_s there.
[[nodiscard]] inline std::string env_var(const char* name) {
#ifdef _MSC_VER
    char* value = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&value, &len, name) != 0 || value == nullptr) {
        return {};
    }
    std::string result(value);
    std::free(value);
    return result;
#else
    const char* value = std::getenv(name);
    return value == nullptr ? std::string{} : std::string(value);
#endif
}

/// The compiler and optimization level, from the predefined macros.
[[nodiscard]] inline std::string compiler_id() {
    std::string id;
#if defined(__clang__)
    id = "clang " + std::to_string(__clang_major__) + "." + std::to_string(__clang_minor__);
#elif defined(__GNUC__)
    id = "gcc " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__);
#elif defined(_MSC_VER)
    id = "msvc " + std::to_string(_MSC_VER);
#else
    id = "unknown";
#endif
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && defined(NDEBUG))
    id += " optimized";
#else
    id += " unoptimized";
#endif
    return id;
}

/// Shortest round-trip text of a finite double. Pre: std::isfinite(v).
[[nodiscard]] inline std::string format_number(double v) {
    if (!std::isfinite(v)) {
        throw std::invalid_argument("reflex-trace: non-finite value");  // W3
    }
    if (v == 0) {
        return "0";  // also folds -0, which JSON readers treat inconsistently
    }
    std::array<char, 32> buf{};
    const auto [end, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    if (ec != std::errc{}) {
        throw std::invalid_argument("reflex-trace: cannot format value");
    }
    return std::string(buf.data(), end);
}

/// A JSON string literal.
[[nodiscard]] inline std::string quote(std::string_view s) {
    std::string out = "\"";
    for (const char c : s) {
        switch (c) {
        case '"':
            out += "\\\"";
            break;
        case '\\':
            out += "\\\\";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                std::array<char, 8> esc{};
                std::snprintf(esc.data(), esc.size(), "\\u%04x", static_cast<unsigned>(c));
                out += esc.data();
            } else {
                out += c;
            }
        }
    }
    return out + "\"";
}

class Trace {
public:
    /// Pre: block and scenario non-empty; dt_sim > 0; dt_trace ≥ 0.
    explicit Trace(Provenance p) : p_(std::move(p)) {
        if (p_.block.empty() || p_.scenario.empty() || !(p_.dt_sim > 0) || !(p_.dt_trace >= 0)) {
            throw std::invalid_argument("reflex-trace: incomplete provenance");
        }
        if (p_.commit.empty()) {
            p_.commit = env_var("REFLEX_TRACE_COMMIT");
        }
        if (p_.commit.empty()) {
            p_.commit = "unknown";
        }
        if (p_.compiler.empty()) {
            p_.compiler = compiler_id();
        }
    }

    /// Declares a channel. Reserved channels (W4) take no unit; any other
    /// channel must name one. Pre: no samples taken yet; name not declared.
    void channel(const std::string& name, std::size_t width, std::string unit = {}) {
        if (!t_.empty() || index_.count(name) != 0 || width == 0) {
            throw std::invalid_argument("reflex-trace: cannot declare channel " + name);
        }
        const std::size_t reserved = reserved_width(name);
        if (reserved != 0 && (width != reserved || !unit.empty())) {
            throw std::invalid_argument("reflex-trace: reserved channel " + name + " has a fixed width and unit");
        }
        if (reserved == 0 && unit.empty()) {
            throw std::invalid_argument("reflex-trace: channel " + name + " needs a unit");
        }
        index_[name] = channels_.size();
        channels_.push_back({name, width, std::move(unit), {}});
    }

    /// Starts sample k at time t; each declared channel then receives exactly
    /// one set() before the next sample() or write().
    void sample(double t) {
        require_complete();
        const std::size_t k = t_.size();
        const bool first_ok = k != 0 || t == 0;
        const bool increasing = k == 0 || t > t_.back();
        const bool uniform =
            p_.dt_trace == 0 || std::abs(t - static_cast<double>(k) * p_.dt_trace) <= 1e-9 * p_.dt_trace;
        if (!std::isfinite(t) || !first_ok || !increasing || !uniform) {
            throw std::invalid_argument("reflex-trace: bad sample time " + format_number(t));  // W1
        }
        t_.push_back(t);
    }

    void set(const std::string& name, std::span<const double> values) {
        const auto it = index_.find(name);
        if (it == index_.end()) {
            throw std::invalid_argument("reflex-trace: undeclared channel " + name);
        }
        Channel& c = channels_[it->second];
        if (t_.empty() || values.size() != c.width || c.data.size() != (t_.size() - 1) * c.width) {
            throw std::invalid_argument("reflex-trace: wrong width or repeated set for " + name);  // W2, W3
        }
        for (const double v : values) {
            if (!std::isfinite(v)) {
                throw std::invalid_argument("reflex-trace: non-finite value in " + name);  // W3
            }
        }
        c.data.insert(c.data.end(), values.begin(), values.end());
    }

    void set(const std::string& name, std::initializer_list<double> values) {
        set(name, std::span<const double>(values.begin(), values.size()));
    }

    void set(const std::string& name, double value) {
        set(name, std::span<const double>(&value, 1));
    }

    void event(double t, std::string label) {
        events_.push_back({t, std::move(label)});
    }

    void check(Check c) {
        if ((c.comparison != "<=" && c.comparison != ">=") || !std::isfinite(c.value) || !std::isfinite(c.limit) ||
            c.id.empty() || c.metric.empty() || c.unit.empty()) {
            throw std::invalid_argument("reflex-trace: malformed check " + c.id);  // W6
        }
        checks_.push_back(std::move(c));
    }

    /// Optional vehicle geometry the 3-D view draws to scale.
    void vehicle_arm_length(double meters) {
        if (!(meters > 0) || !std::isfinite(meters)) {
            throw std::invalid_argument("reflex-trace: bad arm length");
        }
        arm_length_ = meters;
    }

    [[nodiscard]] std::size_t samples() const noexcept {
        return t_.size();
    }

    [[nodiscard]] std::string to_json() const {
        require_complete();
        if (t_.empty()) {
            throw std::invalid_argument("reflex-trace: no samples");
        }
        for (const Event& e : events_) {
            if (!(e.t >= t_.front() && e.t <= t_.back())) {
                throw std::invalid_argument("reflex-trace: event outside the time range: " + e.label);  // W5
            }
        }
        std::string s = "{\n";
        s += "  \"schema\": " + quote(kSchema) + ",\n";
        s += "  \"provenance\": {\n";
        s += "    \"commit\": " + quote(p_.commit) + ", \"block\": " + quote(p_.block) + ",\n";
        s += "    \"scenario\": " + quote(p_.scenario) + ", \"seed\": " + std::to_string(p_.seed) + ",\n";
        s += "    \"compiler\": " + quote(p_.compiler) + ", \"dt_sim\": " + format_number(p_.dt_sim);
        if (p_.dt_trace > 0) {
            s += ", \"dt_trace\": " + format_number(p_.dt_trace);
        }
        s += "\n  },\n";
        s += "  \"frames\": { \"world\": \"z-up\", \"body\": \"x-forward y-left z-up\", \"rotors\": "
             "\"px4-quad-x\" },\n";
        if (arm_length_ > 0) {
            s += "  \"vehicle\": { \"arm_length\": " + format_number(arm_length_) + " },\n";
        }
        s += "  \"t\": [";
        for (std::size_t k = 0; k < t_.size(); ++k) {
            s += (k == 0 ? "" : ", ") + format_number(t_[k]);
        }
        s += "],\n  \"channels\": {";
        for (std::size_t c = 0; c < channels_.size(); ++c) {
            const Channel& ch = channels_[c];
            s += (c == 0 ? "\n" : ",\n") + std::string("    ") + quote(ch.name) + ": [";
            for (std::size_t k = 0; k < t_.size(); ++k) {
                s += k == 0 ? "\n      " : ",\n      ";
                if (ch.width == 1) {
                    s += format_number(ch.data[k]);
                    continue;
                }
                s += "[";
                for (std::size_t i = 0; i < ch.width; ++i) {
                    s += (i == 0 ? "" : ", ") + format_number(ch.data[k * ch.width + i]);
                }
                s += "]";
            }
            s += "\n    ]";
        }
        s += "\n  },\n  \"units\": {";
        bool first = true;
        for (const Channel& ch : channels_) {
            if (!ch.unit.empty()) {
                s += (first ? " " : ", ") + quote(ch.name) + ": " + quote(ch.unit);
                first = false;
            }
        }
        s += first ? "},\n" : " },\n";
        s += "  \"events\": [";
        for (std::size_t i = 0; i < events_.size(); ++i) {
            s += (i == 0 ? "\n" : ",\n") + std::string("    { \"t\": ") + format_number(events_[i].t) +
                 ", \"label\": " + quote(events_[i].label) + " }";
        }
        s += events_.empty() ? "],\n" : "\n  ],\n";
        s += "  \"checks\": [";
        for (std::size_t i = 0; i < checks_.size(); ++i) {
            const Check& c = checks_[i];
            s += (i == 0 ? "\n" : ",\n") + std::string("    { \"id\": ") + quote(c.id) +
                 ", \"metric\": " + quote(c.metric) + ", \"unit\": " + quote(c.unit) +
                 ", \"value\": " + format_number(c.value) + ", \"limit\": " + format_number(c.limit) +
                 ", \"comparison\": " + quote(c.comparison) + ", \"pass\": " + (c.pass() ? "true" : "false") + " }";
        }
        s += checks_.empty() ? "]\n" : "\n  ]\n";
        s += "}\n";
        return s;
    }

    /// Writes the trace to `path`, creating parent directories.
    void write(const std::filesystem::path& path) const {
        const std::string json = to_json();
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }
        std::ofstream out(path, std::ios::binary);
        out << json;
        if (!out) {
            throw std::runtime_error("reflex-trace: cannot write " + path.string());
        }
    }

private:
    struct Channel {
        std::string name;
        std::size_t width;
        std::string unit;
        std::vector<double> data;
    };
    struct Event {
        double t;
        std::string label;
    };

    void require_complete() const {
        for (const Channel& c : channels_) {
            if (c.data.size() != t_.size() * c.width) {
                throw std::invalid_argument("reflex-trace: channel " + c.name + " missing a sample");  // W2
            }
        }
    }

    Provenance p_;
    double arm_length_{0};
    std::vector<double> t_;
    std::vector<Channel> channels_;
    std::map<std::string, std::size_t, std::less<>> index_;
    std::vector<Event> events_;
    std::vector<Check> checks_;
};

/// The output path for a scenario: $REFLEX_TRACE_DIR/<relative>. Empty if the
/// variable is unset (tracing is opt-in, as for the gyro example).
[[nodiscard]] inline std::filesystem::path output_path(const std::filesystem::path& relative) {
    const std::string dir = env_var("REFLEX_TRACE_DIR");
    return dir.empty() ? std::filesystem::path{} : std::filesystem::path(dir) / relative;
}

}  // namespace branes::reflex::trace

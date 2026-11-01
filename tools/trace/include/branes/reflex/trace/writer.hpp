#pragma once

/// reflex-trace/1 writer: the C++ side of the trace format that the docs-site
/// player renders (docs/standards/animation.md, "Trace
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
///   W6: a check's comparison is "<=" or ">=", its value and limit are
///       finite, its id is a requirement ID (e.g. "BB-V3") unique within the
///       trace, and its metric and unit are non-empty. Only a check of the
///       trace can be marked as expected to fail.
///   W7: channel names match [a-z][a-z0-9_]*; at least one channel; event
///       labels are non-empty and event times finite.
///   W8: a scene entity has a unique id matching [a-z0-9][a-z0-9_-]*, a role
///       of the schema's, finite positive dimensions, finite static poses and
///       offsets with unit quaternions, a tree visual on a cylinder only, and
///       a model visual with a uri and a positive scale.
///   W9: at serialization, a pose channel is a declared channel of width 3
///       (position) or 4 (attitude), and an attitude channel's entries are
///       unit quaternions.
/// Together these are the rules of the JSON Schema and validate.js, so a
/// trace the writer accepts passes `npm run traces:check`.
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
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

#if defined(__EMSCRIPTEN__)
#include <emscripten/version.h>
#endif

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
    /// The scenario shows the edge of the block's envelope, so the check is
    /// meant to fail; serialized as "expect": "fail".
    bool expect_fail{false};
    [[nodiscard]] bool pass() const {
        return comparison == "<=" ? value <= limit : value >= limit;
    }
};

// ── The scene (docs/standards/animation, "Trace schema") ─────────────

/// A shape in the entity's frame, centred on its origin: a box of `size`, a
/// cylinder of `radius` and `height` along z, a sphere of `radius`, or a
/// compound of primitives, each at its offset.
struct Shape {
    enum class Kind { box, cylinder, sphere, compound };
    struct Part;

    Kind kind{Kind::box};
    std::array<double, 3> size{};  ///< box [m]
    double radius{0};              ///< cylinder, sphere [m]
    double height{0};              ///< cylinder [m]
    std::vector<Part> parts;       ///< compound

    [[nodiscard]] static Shape box(double x, double y, double z);
    [[nodiscard]] static Shape cylinder(double radius, double height);
    [[nodiscard]] static Shape sphere(double radius);
    [[nodiscard]] static Shape compound(std::vector<Part> parts);
};

/// One primitive of a compound shape, at `position` and `attitude` in the entity's frame.
struct Shape::Part {
    Shape shape;
    std::array<double, 3> position{};
    std::array<double, 4> attitude{1, 0, 0, 0};  ///< [w, x, y, z]
};

inline Shape Shape::box(double x, double y, double z) {
    Shape s;
    s.size = {x, y, z};
    return s;
}

inline Shape Shape::cylinder(double radius, double height) {
    Shape s;
    s.kind = Kind::cylinder;
    s.radius = radius;
    s.height = height;
    return s;
}

inline Shape Shape::sphere(double radius) {
    Shape s;
    s.kind = Kind::sphere;
    s.radius = radius;
    return s;
}

inline Shape Shape::compound(std::vector<Part> parts) {
    Shape s;
    s.kind = Kind::compound;
    s.parts = std::move(parts);
    return s;
}

/// Where an entity is: a fixed position and attitude, or the names of the
/// trace channels that hold them, for an entity that moves.
struct Pose {
    std::variant<std::array<double, 3>, std::string> position;                 ///< [m, world], or a width-3 channel
    std::optional<std::variant<std::array<double, 4>, std::string>> attitude;  ///< [w, x, y, z], or a width-4 channel
};

/// What the player draws in place of the shape: a tree on a cylinder (its
/// trunk the cylinder, its canopy `canopy_radius` wide, 0 for the player's
/// default), or a model (glTF, for later players; drawn as the shape until then).
struct Visual {
    enum class Kind { tree, model };
    Kind kind{Kind::tree};
    double canopy_radius{0};                     ///< tree [m]; 0: the player's default
    std::string uri;                             ///< model
    std::optional<std::array<double, 3>> scale;  ///< model

    [[nodiscard]] static Visual tree(double canopy_radius = 0) {
        Visual v;
        v.canopy_radius = canopy_radius;
        return v;
    }
    [[nodiscard]] static Visual model(std::string uri, std::optional<std::array<double, 3>> scale = std::nullopt) {
        Visual v;
        v.kind = Kind::model;
        v.uri = std::move(uri);
        v.scale = scale;
        return v;
    }
};

/// One object of the scene. The shape is what the simulation tests (an
/// obstacle's collision geometry); a visual only changes how it is drawn.
struct Entity {
    std::string id;    ///< [a-z0-9][a-z0-9_-]*, unique in the trace
    std::string role;  ///< obstacle, goal, gate, boundary, agent or marker
    Shape shape;
    Pose pose;
    std::optional<Visual> visual;
    std::string label;  ///< optional display text
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

/// True if `name` matches [a-z][a-z0-9_]* (W7).
[[nodiscard]] inline bool valid_channel_name(std::string_view name) {
    if (name.empty() || name[0] < 'a' || name[0] > 'z') {
        return false;
    }
    for (const char c : name) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) {
            return false;
        }
    }
    return true;
}

/// True if `id` is a requirement ID, matching [A-Z]+-[A-Z]+[0-9]+ (W6).
[[nodiscard]] inline bool valid_requirement_id(std::string_view id) {
    std::size_t i = 0;
    const auto run = [&](auto pred) {
        const std::size_t start = i;
        while (i < id.size() && pred(id[i])) {
            ++i;
        }
        return i > start;
    };
    const auto upper = [](char c) { return c >= 'A' && c <= 'Z'; };
    const auto digit = [](char c) { return c >= '0' && c <= '9'; };
    if (!run(upper) || i >= id.size() || id[i] != '-') {
        return false;
    }
    ++i;
    return run(upper) && run(digit) && i == id.size();
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
#if defined(__EMSCRIPTEN__)
    id = "emscripten " + std::to_string(__EMSCRIPTEN_MAJOR__) + "." + std::to_string(__EMSCRIPTEN_MINOR__) + "." +
         std::to_string(__EMSCRIPTEN_TINY__) + " wasm32";
#elif defined(__clang__)
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
        if (!t_.empty() || index_.count(name) != 0 || width == 0 || !valid_channel_name(name)) {
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
        if (!std::isfinite(t) || label.empty()) {
            throw std::invalid_argument("reflex-trace: malformed event");  // W7
        }
        events_.push_back({t, std::move(label)});
    }

    void check(Check c) {
        bool duplicate = false;
        for (const Check& other : checks_) {
            duplicate = duplicate || other.id == c.id;
        }
        if ((c.comparison != "<=" && c.comparison != ">=") || !std::isfinite(c.value) || !std::isfinite(c.limit) ||
            !valid_requirement_id(c.id) || duplicate || c.metric.empty() || c.unit.empty()) {
            throw std::invalid_argument("reflex-trace: malformed or duplicate check " + c.id);  // W6
        }
        checks_.push_back(std::move(c));
    }

    /// Marks the check `id` as expected to fail, for a scenario that reuses
    /// another's checks outside the envelope they hold in.
    void expect_failure(std::string_view id) {
        for (Check& c : checks_) {
            if (c.id == id) {
                c.expect_fail = true;
                return;
            }
        }
        throw std::invalid_argument("reflex-trace: no check " + std::string(id));  // W6
    }

    /// Names the block and the scenario source of a scenario that reuses
    /// another's run (for example the same flight with a scene added).
    /// Pre: both non-empty.
    void source(std::string block, std::string scenario) {
        if (block.empty() || scenario.empty()) {
            throw std::invalid_argument("reflex-trace: incomplete provenance");
        }
        p_.block = std::move(block);
        p_.scenario = std::move(scenario);
    }

    /// Adds a scene entity (W8; its pose channels are checked at serialization, W9).
    void entity(Entity e) {
        const auto bad = [&](const std::string& why) {
            return std::invalid_argument("reflex-trace: scene entity " + e.id + ": " + why);
        };
        if (!valid_entity_id(e.id)) {
            throw bad("its id must match [a-z0-9][a-z0-9_-]*");
        }
        for (const Entity& other : entities_) {
            if (other.id == e.id) {
                throw bad("duplicate id");
            }
        }
        static constexpr std::array<std::string_view, 6> kRoles = {
            "obstacle", "goal", "gate", "boundary", "agent", "marker"};
        bool role = false;
        for (const std::string_view r : kRoles) {
            role = role || r == e.role;
        }
        if (!role) {
            throw bad("unknown role " + e.role);
        }
        if (!shape_valid(e.shape, true)) {
            throw bad(
                "a dimension is not finite and > 0, or a compound part is not a primitive at a finite, unit pose");
        }
        if (const auto* p = std::get_if<std::array<double, 3>>(&e.pose.position); p && !finite(*p)) {
            throw bad("its position is not finite");
        }
        if (const auto* name = std::get_if<std::string>(&e.pose.position); name && !valid_channel_name(*name)) {
            throw bad("its position channel name is not a channel name");
        }
        if (e.pose.attitude) {
            if (const auto* q = std::get_if<std::array<double, 4>>(&*e.pose.attitude); q && !unit(*q)) {
                throw bad("its attitude is not a unit quaternion");
            }
            if (const auto* name = std::get_if<std::string>(&*e.pose.attitude); name && !valid_channel_name(*name)) {
                throw bad("its attitude channel name is not a channel name");
            }
        }
        if (e.visual) {
            const Visual& v = *e.visual;
            if (v.kind == Visual::Kind::tree &&
                (e.shape.kind != Shape::Kind::cylinder || !std::isfinite(v.canopy_radius) || v.canopy_radius < 0)) {
                throw bad("a tree is drawn on a cylinder, with a canopy radius ≥ 0");
            }
            if (v.kind == Visual::Kind::model && (v.uri.empty() || (v.scale && !positive(*v.scale)))) {
                throw bad("a model needs a uri, and a scale > 0 if it has one");
            }
        }
        entities_.push_back(std::move(e));
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
        if (t_.empty() || channels_.empty()) {
            throw std::invalid_argument("reflex-trace: no samples or no channels");  // W7
        }
        for (const Event& e : events_) {
            if (!(e.t >= t_.front() && e.t <= t_.back())) {
                throw std::invalid_argument("reflex-trace: event outside the time range: " + e.label);  // W5
            }
        }
        for (const Entity& e : entities_) {
            if (const auto* name = std::get_if<std::string>(&e.pose.position)) {
                pose_channel(e, *name, 3);  // W9
            }
            if (e.pose.attitude) {
                if (const auto* name = std::get_if<std::string>(&*e.pose.attitude)) {
                    const Channel& c = pose_channel(e, *name, 4);  // W9
                    for (std::size_t k = 0; k < t_.size(); ++k) {
                        const std::array<double, 4> q = {
                            c.data[4 * k], c.data[4 * k + 1], c.data[4 * k + 2], c.data[4 * k + 3]};
                        if (!unit(q)) {
                            throw std::invalid_argument("reflex-trace: scene entity " + e.id + ": attitude channel " +
                                                        *name + " has a non-unit entry");  // W9
                        }
                    }
                }
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
        if (!entities_.empty()) {
            s += "  \"scene\": { \"entities\": [";
            for (std::size_t i = 0; i < entities_.size(); ++i) {
                s += (i == 0 ? "\n    " : ",\n    ") + entity_json(entities_[i]);
            }
            s += "\n  ] },\n";
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
                 ", \"comparison\": " + quote(c.comparison) + ", \"pass\": " + (c.pass() ? "true" : "false") +
                 (c.expect_fail ? ", \"expect\": \"fail\" }" : " }");
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

    /// A pose channel of `e`, declared with `width` (W9).
    const Channel& pose_channel(const Entity& e, const std::string& name, std::size_t width) const {
        const auto it = index_.find(name);
        if (it == index_.end() || channels_[it->second].width != width) {
            throw std::invalid_argument("reflex-trace: scene entity " + e.id + ": " + name +
                                        " is not a declared channel of width " + std::to_string(width));
        }
        return channels_[it->second];
    }

    template <std::size_t N>
    [[nodiscard]] static bool finite(const std::array<double, N>& v) {
        for (const double x : v) {
            if (!std::isfinite(x)) {
                return false;
            }
        }
        return true;
    }
    template <std::size_t N>
    [[nodiscard]] static bool positive(const std::array<double, N>& v) {
        for (const double x : v) {
            if (!(std::isfinite(x) && x > 0)) {
                return false;
            }
        }
        return true;
    }
    /// A unit quaternion, within the validator's 1e-6 (validate.js, ATTITUDE_NORM_TOL).
    [[nodiscard]] static bool unit(const std::array<double, 4>& q) {
        return finite(q) && std::abs(std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) - 1) <= 1e-6;
    }
    [[nodiscard]] static bool valid_entity_id(std::string_view id) {
        if (id.empty() || !((id[0] >= 'a' && id[0] <= 'z') || (id[0] >= '0' && id[0] <= '9'))) {
            return false;
        }
        for (const char c : id) {
            if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-')) {
                return false;
            }
        }
        return true;
    }
    /// W8 for a shape; a compound's parts are primitives (top_level false).
    [[nodiscard]] static bool shape_valid(const Shape& s, bool top_level) {
        switch (s.kind) {
        case Shape::Kind::box:
            return positive(s.size);
        case Shape::Kind::cylinder:
            return std::isfinite(s.radius) && s.radius > 0 && std::isfinite(s.height) && s.height > 0;
        case Shape::Kind::sphere:
            return std::isfinite(s.radius) && s.radius > 0;
        case Shape::Kind::compound:
            if (!top_level || s.parts.empty()) {
                return false;
            }
            for (const Shape::Part& p : s.parts) {
                if (!shape_valid(p.shape, false) || !finite(p.position) || !unit(p.attitude)) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    template <std::size_t N>
    [[nodiscard]] static std::string array_json(const std::array<double, N>& v) {
        std::string s = "[";
        for (std::size_t i = 0; i < N; ++i) {
            s += (i == 0 ? "" : ", ") + format_number(v[i]);
        }
        return s + "]";
    }
    [[nodiscard]] static std::string shape_json(const Shape& s) {
        switch (s.kind) {
        case Shape::Kind::box:
            return "{ \"type\": \"box\", \"size\": " + array_json(s.size) + " }";
        case Shape::Kind::cylinder:
            return "{ \"type\": \"cylinder\", \"radius\": " + format_number(s.radius) +
                   ", \"height\": " + format_number(s.height) + " }";
        case Shape::Kind::sphere:
            return "{ \"type\": \"sphere\", \"radius\": " + format_number(s.radius) + " }";
        case Shape::Kind::compound: {
            std::string out = "{ \"type\": \"compound\", \"parts\": [";
            for (std::size_t i = 0; i < s.parts.size(); ++i) {
                const Shape::Part& p = s.parts[i];
                out += (i == 0 ? " " : ", ") + std::string("{ \"shape\": ") + shape_json(p.shape) +
                       ", \"offset\": { \"position\": " + array_json(p.position) +
                       ", \"attitude\": " + array_json(p.attitude) + " } }";
            }
            return out + " ] }";
        }
        }
        return {};
    }
    [[nodiscard]] static std::string entity_json(const Entity& e) {
        std::string s = "{ \"id\": " + quote(e.id) + ", \"role\": " + quote(e.role);
        if (!e.label.empty()) {
            s += ", \"label\": " + quote(e.label);
        }
        s += ", \"shape\": " + shape_json(e.shape) + ", \"pose\": { \"position\": ";
        if (const auto* p = std::get_if<std::array<double, 3>>(&e.pose.position)) {
            s += array_json(*p);
        } else {
            s += quote(std::get<std::string>(e.pose.position));
        }
        if (e.pose.attitude) {
            s += ", \"attitude\": ";
            if (const auto* q = std::get_if<std::array<double, 4>>(&*e.pose.attitude)) {
                s += array_json(*q);
            } else {
                s += quote(std::get<std::string>(*e.pose.attitude));
            }
        }
        s += " }";
        if (e.visual) {
            const Visual& v = *e.visual;
            if (v.kind == Visual::Kind::tree) {
                s += ", \"visual\": { \"kind\": \"tree\"";
                if (v.canopy_radius > 0) {
                    s += ", \"canopy_radius\": " + format_number(v.canopy_radius);
                }
                s += " }";
            } else {
                s += ", \"visual\": { \"kind\": \"model\", \"uri\": " + quote(v.uri);
                if (v.scale) {
                    s += ", \"scale\": " + array_json(*v.scale);
                }
                s += " }";
            }
        }
        return s + " }";
    }

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
    std::vector<Entity> entities_;
};

/// The output path for a scenario: $REFLEX_TRACE_DIR/<relative>. Empty if the
/// variable is unset (tracing is opt-in, as for the gyro example).
[[nodiscard]] inline std::filesystem::path output_path(const std::filesystem::path& relative) {
    const std::string dir = env_var("REFLEX_TRACE_DIR");
    return dir.empty() ? std::filesystem::path{} : std::filesystem::path(dir) / relative;
}

}  // namespace branes::reflex::trace

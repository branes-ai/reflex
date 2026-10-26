// reflex-trace/1 writer (tools/trace): invariants W1–W6 and the serialized form.
#include <branes/reflex/trace/writer.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

using branes::reflex::trace::Check;
using branes::reflex::trace::format_number;
using branes::reflex::trace::Provenance;
using branes::reflex::trace::reserved_width;
using branes::reflex::trace::Trace;

namespace {

Provenance provenance(double dt_trace = 0.01) {
    return {.commit = "abc1234",
            .block = "model/test",
            .scenario = "tests/tools/trace_writer.cpp",
            .seed = 3,
            .compiler = "test",
            .dt_sim = 0.001,
            .dt_trace = dt_trace};
}

bool contains(const std::string& s, const std::string& part) {
    return s.find(part) != std::string::npos;
}

}  // namespace

TEST_CASE("trace writer: numbers round-trip and reject non-finite values", "[trace]") {
    REQUIRE(format_number(0.1) == "0.1");
    REQUIRE(format_number(-0.0) == "0");
    REQUIRE(std::stod(format_number(1.0 / 3.0)) == 1.0 / 3.0);
    REQUIRE_THROWS_AS(format_number(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
    REQUIRE_THROWS_AS(format_number(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST_CASE("trace writer: reserved channel widths", "[trace]") {
    REQUIRE(reserved_width("attitude") == 4);
    REQUIRE(reserved_width("rotor_speeds_setpoint") == 4);
    REQUIRE(reserved_width("roll_setpoint") == 1);
    REQUIRE(reserved_width("roll_integrator") == 0);
}

TEST_CASE("trace writer: channel declarations enforce W4", "[trace]") {
    Trace tr(provenance());
    REQUIRE_THROWS_AS(tr.channel("attitude", 3), std::invalid_argument);         // reserved width
    REQUIRE_THROWS_AS(tr.channel("position", 3, "m"), std::invalid_argument);    // reserved unit
    REQUIRE_THROWS_AS(tr.channel("roll_integrator", 1), std::invalid_argument);  // unit required
    tr.channel("roll_integrator", 1, "N·m");
    REQUIRE_THROWS_AS(tr.channel("roll_integrator", 1, "N·m"), std::invalid_argument);  // duplicate
}

TEST_CASE("trace writer: sample times enforce W1", "[trace]") {
    Trace tr(provenance());
    REQUIRE_THROWS_AS(tr.sample(0.01), std::invalid_argument);  // must start at 0
    tr.sample(0);
    REQUIRE_THROWS_AS(tr.sample(0), std::invalid_argument);      // strictly increasing
    REQUIRE_THROWS_AS(tr.sample(0.015), std::invalid_argument);  // off the dt_trace grid
    tr.sample(0.01);

    Trace nonuniform(provenance(0));
    nonuniform.sample(0);
    nonuniform.sample(0.0137);
    REQUIRE(nonuniform.samples() == 2);
}

TEST_CASE("trace writer: cardinality and width enforce W2 and W3", "[trace]") {
    Trace tr(provenance());
    tr.channel("position", 3);
    REQUIRE_THROWS_AS(tr.set("position", {0, 0, 1}), std::invalid_argument);  // before any sample
    tr.sample(0);
    REQUIRE_THROWS_AS(tr.set("position", {0, 0}), std::invalid_argument);
    REQUIRE_THROWS_AS(tr.set("velocity", {0, 0, 0}), std::invalid_argument);  // undeclared
    REQUIRE_THROWS_AS(tr.set("position", {0, std::nan(""), 1}), std::invalid_argument);
    REQUIRE_THROWS_AS(tr.to_json(), std::invalid_argument);  // position missing at t = 0
    REQUIRE_THROWS_AS(tr.sample(0.01), std::invalid_argument);
    tr.set("position", {0, 0, 1});
    REQUIRE_THROWS_AS(tr.set("position", {0, 0, 1}), std::invalid_argument);  // repeated
    REQUIRE_NOTHROW(tr.to_json());
}

TEST_CASE("trace writer: events and checks enforce W5 and W6", "[trace]") {
    Trace tr(provenance());
    tr.channel("position", 3);
    tr.sample(0);
    tr.set("position", {0, 0, 1});
    tr.sample(0.01);
    tr.set("position", {0, 0, 1});
    REQUIRE_THROWS_AS(tr.check({.id = "BB-V3", .metric = "m", .unit = "1", .value = 1, .limit = 2, .comparison = "<"}),
                      std::invalid_argument);
    tr.event(0.5, "late");
    REQUIRE_THROWS_AS(tr.to_json(), std::invalid_argument);
}

TEST_CASE("trace writer: serialized form", "[trace]") {
    Trace tr(provenance());
    tr.vehicle_arm_length(0.11);
    tr.channel("position", 3);
    tr.channel("roll_integrator", 1, "N·m");
    for (int k = 0; k < 3; ++k) {
        tr.sample(k * 0.01);
        tr.set("position", {0, 0, 1 + k * 0.5});
        tr.set("roll_integrator", k * 0.25);
    }
    tr.event(0.01, "step \"20°\"");
    tr.check({.id = "BB-I2", .metric = "overshoot", .unit = "1", .value = 0.07, .limit = 0.1, .comparison = "<="});
    const std::string json = tr.to_json();
    REQUIRE(contains(json, "\"schema\": \"reflex-trace/1\""));
    REQUIRE(contains(json, "\"commit\": \"abc1234\""));
    REQUIRE(contains(json, "\"dt_trace\": 0.01"));
    REQUIRE(contains(json, "\"vehicle\": { \"arm_length\": 0.11 }"));
    REQUIRE(contains(json, "\"t\": [0, 0.01, 0.02]"));
    REQUIRE(contains(json, "[0, 0, 2]"));
    REQUIRE(contains(json, "\"units\": { \"roll_integrator\": \"N·m\" }"));
    REQUIRE(contains(json, "\"label\": \"step \\\"20°\\\"\""));
    REQUIRE(contains(json, "\"pass\": true"));
}

TEST_CASE("trace writer: names, ids, and labels enforce W6 and W7", "[trace]") {
    using branes::reflex::trace::valid_channel_name;
    using branes::reflex::trace::valid_requirement_id;
    REQUIRE(valid_channel_name("roll_integrator"));
    REQUIRE_FALSE(valid_channel_name("Roll"));
    REQUIRE_FALSE(valid_channel_name("1st"));
    REQUIRE_FALSE(valid_channel_name("roll-rate"));
    REQUIRE(valid_requirement_id("BB-V3"));
    REQUIRE(valid_requirement_id("BB-I12"));
    REQUIRE_FALSE(valid_requirement_id("ANI-12"));  // the schema pattern needs letters after the dash
    REQUIRE_FALSE(valid_requirement_id("BB-3"));
    REQUIRE_FALSE(valid_requirement_id("bb-V3"));
    REQUIRE_FALSE(valid_requirement_id("BB-V3a"));

    Trace tr(provenance());
    REQUIRE_THROWS_AS(tr.channel("Roll", 1, "rad"), std::invalid_argument);
    tr.sample(0);
    REQUIRE_THROWS_AS(tr.to_json(), std::invalid_argument);  // no channels
    REQUIRE_THROWS_AS(tr.event(0, ""), std::invalid_argument);
    const Check c{.id = "BB-V3", .metric = "m", .unit = "1", .value = 1, .limit = 2, .comparison = "<="};
    tr.check(c);
    REQUIRE_THROWS_AS(tr.check(c), std::invalid_argument);  // duplicate id
    Check bad = c;
    bad.id = "V3";
    REQUIRE_THROWS_AS(tr.check(bad), std::invalid_argument);
}

// reflex-trace/1 writer (tools/trace): invariants W1–W9 and the serialized form.
#include <branes/reflex/trace/writer.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

using branes::reflex::trace::Check;
using branes::reflex::trace::Entity;
using branes::reflex::trace::format_number;
using branes::reflex::trace::Pose;
using branes::reflex::trace::Provenance;
using branes::reflex::trace::reserved_width;
using branes::reflex::trace::Shape;
using branes::reflex::trace::Trace;
using branes::reflex::trace::Visual;

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
    REQUIRE_THROWS_AS(tr.expect_failure("BB-V3"), std::invalid_argument);  // no such check
}

TEST_CASE("trace writer: an expected failure is serialized", "[trace]") {
    Trace tr(provenance());
    tr.channel("position", 3);
    tr.sample(0);
    tr.set("position", {0, 0, 1});
    tr.check({.id = "BB-I2", .metric = "m", .unit = "1", .value = 3, .limit = 2, .comparison = "<="});
    tr.check({.id = "BB-C2", .metric = "n", .unit = "1", .value = 1, .limit = 2, .comparison = "<="});
    REQUIRE_FALSE(contains(tr.to_json(), "\"expect\""));
    tr.expect_failure("BB-I2");
    const std::string json = tr.to_json();
    REQUIRE(contains(json, "\"pass\": false, \"expect\": \"fail\" }"));
    REQUIRE(contains(json, "\"pass\": true }"));
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

namespace {

/// A trace of two samples with a position channel and an agent's pose channels.
Trace two_samples() {
    Trace tr(provenance());
    tr.channel("position", 3);
    tr.channel("agent_position", 3, "m");
    tr.channel("agent_attitude", 4, "1");
    for (int k = 0; k < 2; ++k) {
        tr.sample(k * 0.01);
        tr.set("position", {0, 0, 1});
        tr.set("agent_position", {2.0 + k, 0, 1});
        tr.set("agent_attitude", {1, 0, 0, 0});
    }
    return tr;
}

Entity tree(const std::string& id) {
    return {.id = id,
            .role = "obstacle",
            .shape = Shape::cylinder(0.3, 6),
            .pose = {.position = std::array<double, 3>{4, 1, 3}, .attitude = std::nullopt},
            .visual = Visual::tree(),
            .label = "obstacle 3"};
}

}  // namespace

TEST_CASE("trace writer: the scene's serialized form", "[trace][scene]") {
    Trace tr = two_samples();
    tr.entity(tree("tree-3"));
    tr.entity({.id = "gate-1",
               .role = "gate",
               .shape = Shape::compound({{Shape::box(0.1, 0.1, 2), {0, -1, 0}, {1, 0, 0, 0}},
                                         {Shape::box(0.1, 0.1, 2), {0, 1, 0}, {1, 0, 0, 0}}}),
               .pose = {.position = std::array<double, 3>{5, 0, 1}, .attitude = std::array<double, 4>{1, 0, 0, 0}},
               .visual = std::nullopt,
               .label = {}});
    tr.entity({.id = "agent",
               .role = "agent",
               .shape = Shape::sphere(0.2),
               .pose = {.position = std::string("agent_position"), .attitude = std::string("agent_attitude")},
               .visual = Visual::model("models/balloon.glb", std::array<double, 3>{1, 1, 1}),
               .label = {}});
    const std::string json = tr.to_json();
    REQUIRE(contains(json, "\"scene\": { \"entities\": ["));
    REQUIRE(contains(json,
                     "{ \"id\": \"tree-3\", \"role\": \"obstacle\", \"label\": \"obstacle 3\", "
                     "\"shape\": { \"type\": \"cylinder\", \"radius\": 0.3, \"height\": 6 }, "
                     "\"pose\": { \"position\": [4, 1, 3] }, \"visual\": { \"kind\": \"tree\" } }"));
    REQUIRE(contains(json,
                     "\"shape\": { \"type\": \"compound\", \"parts\": [ { \"shape\": { \"type\": \"box\", "
                     "\"size\": [0.1, 0.1, 2] }, \"offset\": { \"position\": [0, -1, 0], "
                     "\"attitude\": [1, 0, 0, 0] } }, "));
    REQUIRE(contains(json, "\"pose\": { \"position\": \"agent_position\", \"attitude\": \"agent_attitude\" }"));
    REQUIRE(
        contains(json, "\"visual\": { \"kind\": \"model\", \"uri\": \"models/balloon.glb\", \"scale\": [1, 1, 1] }"));
    // A trace without a scene has no scene key, as before.
    REQUIRE_FALSE(contains(two_samples().to_json(), "\"scene\""));
}

TEST_CASE("trace writer: scene entities enforce W8", "[trace][scene]") {
    Trace tr = two_samples();
    tr.entity(tree("tree-3"));
    REQUIRE_THROWS_AS(tr.entity(tree("tree-3")), std::invalid_argument);  // duplicate id
    REQUIRE_THROWS_AS(tr.entity(tree("Tree")), std::invalid_argument);    // id pattern
    REQUIRE_THROWS_AS(tr.entity(tree("-tree")), std::invalid_argument);
    Entity e = tree("tree-4");
    e.role = "tree";
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // unknown role
    e = tree("tree-4");
    e.shape = Shape::cylinder(0, 6);
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // zero radius
    e.shape = Shape::box(1, 1, std::numeric_limits<double>::infinity());
    e.visual = std::nullopt;
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // infinite size
    e.shape = Shape::box(1, 1, 1);
    e.visual = Visual::tree();
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // a tree on a box
    e.visual = Visual::model("");
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // a model without a uri
    e.visual = std::nullopt;
    e.shape =
        Shape::compound({{Shape::compound({{Shape::sphere(1), {0, 0, 0}, {1, 0, 0, 0}}}), {0, 0, 0}, {1, 0, 0, 0}}});
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // a compound inside a compound
    e.shape = Shape::compound({{Shape::sphere(1), {0, 0, 0}, {0.9, 0, 0, 0}}});
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // a part's attitude not unit
    e.shape = Shape::sphere(1);
    e.pose.attitude = std::array<double, 4>{1, 0.1, 0, 0};
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // the pose's attitude not unit
    e.pose = {.position = std::string("Agent"), .attitude = std::nullopt};
    REQUIRE_THROWS_AS(tr.entity(e), std::invalid_argument);  // not a channel name
    e.pose = {.position = std::array<double, 3>{0, 0, 0}, .attitude = std::nullopt};
    tr.entity(e);  // all fixed
}

TEST_CASE("trace writer: scene pose channels enforce W9 at serialization", "[trace][scene]") {
    const auto with = [](Pose pose) {
        Trace tr = two_samples();
        tr.entity({.id = "agent",
                   .role = "agent",
                   .shape = Shape::sphere(0.2),
                   .pose = std::move(pose),
                   .visual = std::nullopt,
                   .label = {}});
        return tr;
    };
    REQUIRE_NOTHROW(with({.position = std::string("agent_position"), .attitude = std::nullopt}).to_json());
    REQUIRE_THROWS_AS(with({.position = std::string("intruder"), .attitude = std::nullopt}).to_json(),
                      std::invalid_argument);  // not declared
    REQUIRE_THROWS_AS(with({.position = std::string("agent_attitude"), .attitude = std::nullopt}).to_json(),
                      std::invalid_argument);  // width 4 for a position
    REQUIRE_THROWS_AS(with({.position = std::array<double, 3>{0, 0, 0}, .attitude = std::string("position")}).to_json(),
                      std::invalid_argument);  // width 3 for an attitude

    Trace bad(provenance());
    bad.channel("spin", 4, "1");
    bad.sample(0);
    bad.set("spin", {1, 1, 0, 0});  // not a unit quaternion
    bad.entity({.id = "agent",
                .role = "agent",
                .shape = Shape::sphere(0.2),
                .pose = {.position = std::array<double, 3>{0, 0, 0}, .attitude = std::string("spin")},
                .visual = std::nullopt,
                .label = {}});
    REQUIRE_THROWS_AS(bad.to_json(), std::invalid_argument);
}

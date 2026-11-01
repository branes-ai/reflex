// Writes $REFLEX_TRACE_DIR/player/scene_square.json: the cascade's waypoint
// square (cascade/waypoint.hpp, at its defaults) with a scene around it, the
// trace player's example of the scene (docs/standards/trace-player.mdx).
//
// The flight is the waypoint scenario's, unchanged, with its check. The scene:
//  - the boundary, a 16 × 16 × 8 m box around the square;
//  - six trees, cylinders drawn as trees, none within 2 m of the path, one in
//    the middle of the square;
//  - a rock, a box with no visual, drawn in the obstacle colour;
//  - a gate on the third leg, four bars around a 2 × 2 m opening the path
//    runs through;
//  - the goal, a sphere at the corner the square starts and ends at;
//  - the position setpoint, a marker that moves: a thin box along the
//    attitude target's x (its heading), its pose read from the
//    setpoint_position and target_attitude channels.

#include "../cascade/waypoint.hpp"

#include <array>
#include <optional>
#include <string>

namespace {

namespace trace = branes::reflex::trace;

/// A tree standing on the ground at (x, y): its cylinder is centred half its height up.
trace::Entity tree(const std::string& id, double x, double y, double radius, double height) {
    return {.id = id,
            .role = "obstacle",
            .shape = trace::Shape::cylinder(radius, height),
            .pose = {.position = std::array<double, 3>{x, y, height / 2}, .attitude = std::nullopt},
            .visual = trace::Visual::tree(),
            .label = {}};
}

trace::Entity fixed(std::string id, std::string role, trace::Shape shape, std::array<double, 3> at) {
    return {.id = std::move(id),
            .role = std::move(role),
            .shape = std::move(shape),
            .pose = {.position = at, .attitude = std::nullopt},
            .visual = std::nullopt,
            .label = {}};
}

}  // namespace

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    namespace cw = branes::reflex::trace::scenarios::cascade_waypoint;
    return qt::guarded("scene_square", [] {
        trace::Trace tr = cw::run({});
        tr.source("trace-player", "tools/trace/scenarios/player/scene_square.cpp");
        tr.entity(fixed("boundary", "boundary", trace::Shape::box(16, 16, 8), {5, 5, 4}));
        tr.entity(tree("tree-1", 5, -2.5, 0.25, 7));
        tr.entity(tree("tree-2", 12.5, 3, 0.3, 8));
        tr.entity(tree("tree-3", 12, 8, 0.2, 6));
        tr.entity(tree("tree-4", -2.5, 6, 0.35, 8.5));
        tr.entity(tree("tree-5", -2, -2, 0.2, 6));
        tr.entity(tree("tree-6", 5, 5, 0.4, 9));
        tr.entity(fixed("rock", "obstacle", trace::Shape::box(1.5, 1, 1), {8, 12.5, 0.5}));
        // The third leg runs along −x at y = 10, z = 5: the gate's opening faces x.
        const auto bar = [](double y, double z, double dy, double dz) {
            return trace::Shape::Part{trace::Shape::box(0.1, dy, dz), {0, y, z}, {1, 0, 0, 0}};
        };
        tr.entity(fixed(
            "gate",
            "gate",
            trace::Shape::compound(
                {bar(-1.05, 0, 0.1, 2.2), bar(1.05, 0, 0.1, 2.2), bar(0, -1.05, 2.2, 0.1), bar(0, 1.05, 2.2, 0.1)}),
            {5, 10, 5}));
        tr.entity(fixed("goal", "goal", trace::Shape::sphere(0.5), {0, 0, 5}));
        tr.entity({.id = "setpoint",
                   .role = "marker",
                   .shape = trace::Shape::box(0.3, 0.04, 0.04),
                   .pose = {.position = std::string("setpoint_position"), .attitude = std::string("target_attitude")},
                   .visual = std::nullopt,
                   .label = "the position setpoint, with the attitude target"});
        return qt::write(tr, "player/scene_square.json");
    });
}

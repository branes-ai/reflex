// Writes $REFLEX_TRACE_DIR/cascade/waypoint.json: the scenario in waypoint.hpp
// with its default parameters.

#include "waypoint.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("waypoint", [] {
        return qt::write(branes::reflex::trace::scenarios::cascade_waypoint::run({}), "cascade/waypoint.json");
    });
}

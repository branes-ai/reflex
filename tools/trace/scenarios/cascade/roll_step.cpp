// Writes $REFLEX_TRACE_DIR/cascade/roll_step.json: the scenario in roll_step.hpp
// with its default parameters.

#include "roll_step.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("roll_step", [] {
        return qt::write(branes::reflex::trace::scenarios::roll_step::run({}), "cascade/roll_step.json");
    });
}

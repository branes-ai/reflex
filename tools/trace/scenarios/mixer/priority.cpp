// Writes $REFLEX_TRACE_DIR/mixer/priority.json: the scenario in priority.hpp
// with its default parameters.

#include "priority.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("priority", [] {
        return qt::write(branes::reflex::trace::scenarios::mixer_priority::run({}), "mixer/priority.json");
    });
}

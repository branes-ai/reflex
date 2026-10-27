// Writes $REFLEX_TRACE_DIR/quadrotor/tumble.json: the scenario in tumble.hpp with
// its default parameters.

#include "tumble.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded(
        "tumble", [] { return qt::write(branes::reflex::trace::scenarios::tumble::run({}), "quadrotor/tumble.json"); });
}

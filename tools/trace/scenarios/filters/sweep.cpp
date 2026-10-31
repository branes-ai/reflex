// Writes $REFLEX_TRACE_DIR/filters/sweep.json: the scenario in sweep.hpp with
// its default parameters.

#include "sweep.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("sweep", [] {
        return qt::write(branes::reflex::trace::scenarios::filters_sweep::run({}), "filters/sweep.json");
    });
}

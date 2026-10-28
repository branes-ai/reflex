// Writes $REFLEX_TRACE_DIR/pid/derivative_filter.json: the scenario in derivative_filter.hpp with its
// default parameters.

#include "derivative_filter.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("derivative_filter", [] {
        return qt::write(branes::reflex::trace::scenarios::pid_derivative_filter::run({}),
                         "pid/derivative_filter.json");
    });
}

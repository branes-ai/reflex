// Writes $REFLEX_TRACE_DIR/pid/terms.json: the scenario in terms.hpp with its
// default parameters.

#include "terms.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded(
        "terms", [] { return qt::write(branes::reflex::trace::scenarios::pid_terms::run({}), "pid/terms.json"); });
}

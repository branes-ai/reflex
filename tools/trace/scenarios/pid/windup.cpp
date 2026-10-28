// Writes $REFLEX_TRACE_DIR/pid/windup.json: the scenario in windup.hpp with its
// default parameters.

#include "windup.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded(
        "windup", [] { return qt::write(branes::reflex::trace::scenarios::pid_windup::run({}), "pid/windup.json"); });
}

// Writes $REFLEX_TRACE_DIR/quadrotor/motor_lag.json: the scenario in motor_lag.hpp with
// its default parameters.

#include "motor_lag.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("motor_lag", [] {
        return qt::write(branes::reflex::trace::scenarios::motor_lag::run({}), "quadrotor/motor_lag.json");
    });
}

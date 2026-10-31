// Writes $REFLEX_TRACE_DIR/filters/low_rate.json: the throttle sweep of
// sweep.hpp at a 1 kHz gyro rate, RpmNotchConfig's default.
//
// At 1 kHz the notches can sit no higher than 0.45·fs = 450 Hz, and the
// sampled signal holds nothing above 500 Hz: RP-1's second harmonic passes
// 450 Hz just above hover, and at the top of the sweep even the fastest
// fundamental does. A harmonic above 500 Hz aliases, appearing at 1000 Hz
// minus its frequency, where no notch follows it. The sweep's BB-I2 check
// fails here: this is outside the block's envelope, which the deployment
// guide shows.

#include "sweep.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    namespace sw = branes::reflex::trace::scenarios::filters_sweep;
    return qt::guarded("low_rate", [] { return qt::write(sw::sweep({}, sw::kGyro1k), "filters/low_rate.json"); });
}

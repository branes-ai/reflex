// Writes $REFLEX_TRACE_DIR/mixer/clip_vs_priority.json: the scenario in
// clip_vs_priority.hpp with its default parameter.

#include "clip_vs_priority.hpp"

int main() {
    namespace qt = branes::reflex::trace::quadrotor;
    return qt::guarded("clip_vs_priority", [] {
        return qt::write(branes::reflex::trace::scenarios::mixer_clip_vs_priority::run({}),
                         "mixer/clip_vs_priority.json");
    });
}

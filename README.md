# reflex

Branes.AI platform components that implement the **involuntary nervous system** of an autonomous machine: the fast, deterministic control and safety loops that keep it stable and alive regardless of what the higher-level "brain" is doing.

## Vision

Mammals split their nervous system into two very different kinds of processing:

- **The cortex** handles deliberate, high-level work: perceiving the world, building a model of it, and deciding where to go. It is powerful, but it is slow and it can be wrong.
- **The involuntary nervous system** (spinal reflexes, the brainstem, the autonomic system) works without conscious thought. It keeps you balanced, pulls your hand off a hot stove before you feel the pain, and keeps your heart beating. It is fast, it is bounded, and it always runs.

The Branes.AI platform follows the same split:

| Repo | Biological analogue | Responsibility | Time scale |
|---|---|---|---|
| [`cortex`](https://github.com/branes-ai/cortex) | Cerebral cortex | Perception (VIO, SLAM, SfM), world building (3D scene graphs), path planning | ~5–100 Hz, best effort |
| **`reflex`** (this repo) | Brainstem, spinal cord, autonomic system | Stabilization, trajectory tracking, optimal control, fault detection and recovery, safety envelopes | ~100 Hz–8 kHz, hard real-time |
| [`racing-drone`](https://github.com/branes-ai/racing-drone) | The whole organism | Mission and flight controller for an autonomous racing drone, assembled from `cortex` and `reflex` components | — |

`cortex` tells the vehicle *where it should go*. `reflex` makes sure it *gets there without falling out of the sky*, and takes over when something goes wrong: a motor fails, a sensor drops out, or the planner sends a command that would leave the safe flight envelope.

## Design principles

- **Determinism over throughput.** Every control loop has a fixed rate and a worst-case execution time that can be bounded. Nothing on the hot path allocates, blocks, or waits on the deliberative layer.
- **The reflex layer never depends on the cortex to stay safe.** If perception or planning stalls, crashes, or produces garbage, `reflex` keeps flying, holds position, or lands. Commands from `cortex` are setpoints to track, never something the vehicle needs in order to stay stable.
- **Safety is a control problem, not an afterthought.** Constraint handling, actuator-failure recovery, and envelope protection belong in the controller formulation itself. That is why the roadmap goes past PID.
- **Hardware-agnostic algorithms, hardware-specific execution.** Controllers are written against clean state/actuator interfaces so the same algorithm runs in simulation (SITL), hardware-in-the-loop (HIL), and on the target flight computer.

## Roadmap

The control stack grows from classical to optimal to fault-tolerant control.

### Phase 1: Classical flight control (PID)

- Cascaded rate → attitude → velocity → position PID loops for multirotors
- Gyro/accel filtering (low-pass, notch, RPM-based dynamic notch)
- Motor mixing, thrust/torque allocation, and actuator saturation handling
- Anti-windup, feed-forward, and gain scheduling
- SITL harness with a rigid-body multirotor dynamics model

### Phase 2: Linear optimal control (LQR / linear MPC)

- Linearized multirotor models and LQR baselines
- Linear MPC with explicit state and input constraints (thrust limits, attitude limits, rate limits)
- Real-time QP solvers with warm starting and bounded iteration counts

### Phase 3: Non-linear MPC

- Full non-linear rigid-body dynamics on SO(3) / SE(3)
- Non-linear MPC for aggressive, time-optimal trajectory tracking: the regime a racing drone lives in
- Real-time iteration (RTI) schemes and SQP solvers suited to kHz-rate control on embedded hardware

### Phase 4: Fault tolerance and safety

- Actuator and sensor fault detection and isolation (FDI)
- Control reallocation after rotor/motor loss (for example, stable reduced-authority flight on three rotors)
- Safety envelopes and command filtering between `cortex` and the actuators
- Graceful degradation ladder: full mission → hold → return/land → terminal failsafe
- Fault-injection testing in SITL and HIL

## Relationship to the racing drone

[`racing-drone`](https://github.com/branes-ai/racing-drone) is the first integration target and the forcing function for this repo. Autonomous drone racing pushes every part of the control stack to its limit: high-rate inner loops, aggressive near-saturation maneuvers, tight latency budgets between perception and actuation, and no margin for a controller that hesitates. The goal of `reflex` is to provide enough high-performance optimal-control components (from a well-tuned PID baseline up to non-linear MPC) to build a competitive racing drone, and to carry those same components forward into safety-critical platforms where fault-tolerant control is a requirement, not a luxury.

## Status

Early stage. The repository has just been created. Build system, directory layout, and first PID components will follow.

## License

MIT. See [LICENSE](LICENSE).

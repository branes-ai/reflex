---
title: Roadmap
description: From classical PID to fault-tolerant non-linear MPC.
---

The control stack grows from classical to optimal to fault-tolerant control.
The first integration target is the
[autonomous racing drone](https://github.com/branes-ai/racing-drone).

## Phase 1: Classical flight control (PID)

- [x] Discrete PID with derivative-on-measurement, derivative filtering,
      integral clamping, and conditional-integration anti-windup
- [ ] Cascaded rate → attitude → velocity → position loops for multirotors
- [ ] Gyro/accel filtering (low-pass, notch, RPM-based dynamic notch)
- [ ] Motor mixing, thrust/torque allocation, actuator saturation handling
- [ ] SITL harness with a rigid-body multirotor dynamics model

## Phase 2: Linear optimal control

- [ ] LQR baselines on linearized multirotor models
- [ ] Linear MPC with state and input constraints
- [ ] Real-time QP solvers with warm starting and bounded iteration counts

## Phase 3: Non-linear MPC

- [ ] Non-linear rigid-body dynamics on SO(3) / SE(3)
- [ ] NMPC for aggressive, time-optimal trajectory tracking
- [ ] Real-time iteration (RTI) / SQP solvers for kHz-rate control

## Phase 4: Fault tolerance and safety

- [ ] Actuator and sensor fault detection and isolation
- [ ] Control reallocation after rotor/motor loss
- [ ] Safety envelopes and command filtering
- [ ] Graceful degradation: mission → hold → return/land → terminal failsafe

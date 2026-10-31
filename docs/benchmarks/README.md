# Benchmarks

Measured performance of reflex's blocks: the cost of an update, and how it scales with the scalar type and the problem size. Each page states what was measured, on which machine (CPU, OS), with which compiler and preset, at which commit, and how to reproduce it.

The measurements come from the micro-benchmarks in [`bench/`](../../bench/), one executable per block (`pid_bench`, `mixer_bench`, `cascade_bench`, `filter_bench`, `quadrotor_bench`), built with a Release preset as [Build & Test](../getting-started/build-and-test.md) describes. Timings depend on the machine, so they are characterization (BB-C4 in the [building block standard](../standards/building-blocks.md)), not CI gates.

Pages are named after the block or the comparison, such as `pid.md` or `float-vs-double.md`. None yet.

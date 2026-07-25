# Time-Optimal Path Parameterization (TOPP) — Overview

## What it is
Given a **fixed geometric path** (the shape the end-effector or joints must follow) and the robot's
**actuator limits**, TOPP computes the fastest possible timing law `s(t)` to traverse that path
without violating any velocity or torque bound. It answers "how fast can I run *this* curve?" — the
geometry is frozen, only the speed along it is optimized.

## Why it matters (embedded)
Separating *where* to go (path planning) from *how fast* to go (parameterization) is a powerful
decomposition. Once a collision-free path exists, TOPP squeezes maximum throughput out of the
hardware — critical for cycle-time-bound industrial robots — while provably respecting drive
saturation, so the plan never asks for torque the motors cannot deliver.

## How it works (intuition)
Reparameterize dynamics in terms of the scalar path coordinate `s`: each joint torque becomes affine
in `s̈` and `ṡ²`. This collapses the high-dimensional problem to a 2-D `(s, ṡ)` phase plane. The
**maximum velocity curve** (MVC) marks the fastest `ṡ` still feasible at each `s`. Modern TOPP-RA
then runs a backward "controllable set" pass and a forward "reachable set" pass over a grid, each
step solving a tiny 1-D linear program — numerically robust, linear-time, and free of the fragile
switch-point search in Bobrow's original phase-plane method.

## Key parameters
- **Path geometry** — `q(s)` and its first/second derivatives along `s ∈ [0, 1]`.
- **Velocity & torque limits** — per-joint bounds, torques via inverse dynamics (RNEA).
- **Grid resolution** — number of `s` samples; trades accuracy against planning cost.

## Reference
J. Bobrow, S. Dubowsky, J. Gibson, "Time-Optimal Control of Robotic Manipulators Along Specified
Paths," *IJRR* 4(3), 1985; Q.-C. Pham, "A General, Fast, and Robust Implementation of the
Time-Optimal Path Parameterization Algorithm" (TOPP-RA), *IEEE T-RO*, 2014.

## See also
`CartesianSlerpInterpolation` (supplies the path), `RecursiveNewtonEuler` (torque limits),
`SCurveProfile` (heuristic limit-respecting alternative).

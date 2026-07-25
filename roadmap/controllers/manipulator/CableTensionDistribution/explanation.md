# Cable Tension Distribution — Overview

## What it is
The force-allocation step for a cable-driven robot: given a desired wrench (force + torque) on the
moving platform, compute a set of **non-negative** cable tensions that produce exactly that wrench.
Because cables can only *pull*, and there are usually more cables than task dimensions, this is a
constrained optimisation, not a plain linear solve.

## Why it matters (embedded)
Cable robots — warehouse cranes, camera rigs (SkyCam), tendon-driven hands, large 3D printers —
actuate through tension only. Every control cycle must hand the winches a feasible, bounded tension
vector; a negative or over-limit request is physically impossible and can slacken a cable or snap
it. The allocator runs in the real-time loop on the same controller that computes the wrench.

## How it works (intuition)
The **structure matrix** `A` maps cable tensions to platform wrench (`A·t = w`). With more cables
than task DOF, infinitely many tension sets produce the same wrench — the extra freedom is *internal
pretension*. A small quadratic program picks the tension vector closest to the mid-range value while
satisfying `A·t = w` and staying within `[tMin, tMax]`. Centring the tensions keeps every cable
comfortably taut, maximising the margin against both going slack and overloading.

## Key parameters
- **tMin** — minimum tension (> 0) so cables never go slack.
- **tMax** — maximum tension set by winch/cable strength.
- **structure matrix A = −Jᵀ** — geometry of cable directions and attachment points.
- **objective centre (tMid)** — value the QP biases toward for maximum disturbance margin.

## Reference
T. Bruckmann, A. Pott (eds.), *Cable-Driven Parallel Robots*, Springer, 2013 (Pott
tension-distribution methods).

## See also
`SpatialJacobian` (#M8, supplies `A = −Jᵀ`), `Mpc` (the reused bounded-QP solver),
`OperationalSpaceControl` (#M18, wrench-to-torque mapping for rigid arms).

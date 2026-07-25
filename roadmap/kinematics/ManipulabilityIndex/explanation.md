# Manipulability Index — Overview

## What it is
A single number that scores how well an arm can move in its current posture. Yoshikawa's measure,
`w = √det(J Jᵀ)`, is the volume of the "velocity ellipsoid" — the set of tool velocities reachable by
unit-norm joint rates. Big `w` means dexterous; `w = 0` means the arm is at a singularity.

## Why it matters (embedded)
Singularities are where Jacobian-based controllers blow up: joint rates rocket toward infinity for a
finite tool motion. A cheap scalar that flags "you are getting close" lets a real-time controller slow
down, switch strategies, or damp the inverse *before* the actuators saturate. It is also the objective
a redundant arm optimizes in its null space to stay out of trouble.

## How it works (intuition)
Map the unit sphere of joint velocities through `J` and it becomes an ellipsoid in task space. The
ellipsoid's principal axes are the singular values of `J`; its volume is their product, which equals
`√det(J Jᵀ)`. When the arm nears a singularity one axis collapses — one singular value goes to zero —
so the volume, and hence `w`, drops to zero. The ratio of the longest to shortest axis (the condition
number) tells you how *lopsided* the reachable set is, a finer warning than volume alone.

## Key parameters
- **the Jacobian `J`** — supplied by `SpatialJacobian` at the current joint configuration.
- **singularity threshold `eps`** — below which `NearSingular` trips.
- **square vs redundant** — picks `|det J|` versus the Gram-determinant form.

## Reference
T. Yoshikawa, "Manipulability of Robotic Mechanisms," *Int. J. Robotics Research*, 4(2), 1985.

## See also
`SpatialJacobian` (M8, the input), `SingularValueDecomposition` (item 43, robust axes + conditioning),
`RedundancyResolution` (M14, which maximizes this in the null space).

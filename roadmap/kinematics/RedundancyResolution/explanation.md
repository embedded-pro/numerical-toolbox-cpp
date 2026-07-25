# Redundancy Resolution — Overview

## What it is
A way to use a robot that has *more* joints than the task needs. A 7-DOF arm reaching a 6-DOF pose has
one spare degree of freedom; redundancy resolution splits the joint motion into a **primary** part that
achieves the task and a **secondary** part, living in the Jacobian's null space, that pursues an extra
goal without disturbing the tool.

## Why it matters (embedded)
Extra DOF are what let a robot dodge its own joint limits, avoid obstacles, or stay away from
singularities *while still doing the job*. The formula `q̇ = J⁺ẋ + (I − J⁺J)q̇₀` packages that into one
matrix expression cheap enough to run every control tick — the difference between an arm that jams at a
joint stop and one that gracefully reconfigures around it.

## How it works (intuition)
The pseudo-inverse `J⁺` gives the smallest joint motion that produces the desired tool velocity — the
primary term. The projector `I − J⁺J` filters *any* vector down to the component the tool cannot
feel: push the joints with it and the end-effector holds still. So you pick a secondary joint velocity
`q̇₀` — typically the downhill direction of some cost like "distance from joint limits" or
"manipulability" — run it through the projector, and add it on. The task is untouched; the spare
freedom does useful work.

## Key parameters
- **task rate `ẋ`** — the desired 6-vector end-effector velocity (primary objective).
- **secondary rate `q̇₀`** — gradient of the objective the null space should optimize.
- **damping λ** — singularity-robustness of the pseudo-inverse.

## Reference
A. Liégeois, "Automatic Supervisory Control of the Configuration and Behavior of Multibody
Mechanisms," *IEEE Trans. Systems, Man, and Cybernetics*, 7(12), 1977.

## See also
`SpatialJacobian` (M8, supplies `J`), `SingularValueDecomposition` / `QrDecomposition` (items 43/27,
robust `J⁺`), `ManipulabilityIndex` (M11, a common secondary objective).

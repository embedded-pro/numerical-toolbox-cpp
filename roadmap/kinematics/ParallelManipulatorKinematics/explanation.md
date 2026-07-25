# Parallel Manipulator Kinematics — Overview

## What it is
Kinematics for robots whose moving platform is held by several legs in parallel — the pick-and-place
**Delta** (three arms) and the six-legged **Stewart-Gough** hexapod. Given a platform pose it finds the
leg lengths (inverse), and given the leg lengths it finds the platform pose (forward).

## Why it matters (embedded)
Parallel robots are stiff, fast, and accurate — the workhorses of high-speed packaging and motion
platforms. Their control flips the usual serial-arm effort: computing the leg commands from a desired
pose is a trivial, closed-form, per-leg calculation that fits any real-time loop, exactly what the
servo layer needs on every tick.

## How it works (intuition)
For the **inverse** problem each leg is independent: transform its platform anchor into the base frame,
subtract its base anchor, and the length is just the distance between those two points — no coupling, no
iteration. The **forward** problem is the hard one, because moving one leg moves the whole platform and
thus every other leg. It has no general closed form, so you solve it with Newton's method: guess a
pose, compute what leg lengths it *would* produce, and correct the pose until those match the measured
lengths. Several platform poses can produce the same leg lengths (assembly modes), so the initial guess
picks which one you land on. The three-legged Delta is special enough to admit a closed-form forward
solve by intersecting spheres.

## Key parameters
- **base and platform anchor points** — the fixed geometry of the mechanism.
- **leg lengths** — the actuated variables (prismatic legs).
- **forward-solve guess** — selects the assembly mode and warm-starts Newton.

## Reference
J.-P. Merlet, *Parallel Robots*, 2nd ed. (2006); R. Clavel, "Delta, a Fast Robot with Parallel
Geometry," *Int. Symp. on Industrial Robots*, 1990.

## See also
`SE3Transform` (M6, platform pose), `LuDecomposition` / `GaussianElimination` (item 28, the Newton
step), `ManipulabilityIndex` (M11, singularity monitoring).

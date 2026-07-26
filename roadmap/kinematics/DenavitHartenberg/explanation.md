# Denavit-Hartenberg Parameters — Overview

## What it is
A minimal four-number recipe — `(a, α, d, θ)` — that describes where each link of a robot arm sits
relative to the previous one. Each link's four parameters generate one 4×4 homogeneous transform, and
multiplying them down the chain gives the forward kinematics.

## Why it matters (embedded)
DH is the *lingua franca* of industrial robotics: nearly every arm's datasheet ships a DH table.
Encoding a manipulator as `N×4` constants (plus a joint-type flag) is the most compact possible model,
and the per-link transform is a handful of trig calls — cheap enough for a servo loop on a microcontroller.

## How it works (intuition)
The convention forces every joint axis onto its frame's `z`-axis and every common normal onto the
`x`-axis. That discipline collapses the six numbers of a general rigid transform down to four: two
describe the joint (`d` slides along `z`, `θ` rotates about `z`) and two describe the link that follows
(`a` slides along `x`, `α` twists about `x`). One of `θ`/`d` is the moving joint variable; the other
three are fixed geometry. Chaining the link transforms walks the frame from the base out to the tool.

## Key parameters
- **a (link length), α (link twist)** — fixed geometry of the link.
- **d (link offset), θ (joint angle)** — one is the joint variable, chosen by the joint type.
- **convention** — standard (distal) vs modified (proximal) DH place the frame differently.
- **joint type** — revolute (θ varies) or prismatic (d varies).

## Reference
J. J. Craig, *Introduction to Robotics: Mechanics and Control*, 4th ed., Ch. 3 (the DH convention).

## See also
`SE3Transform` (M6, the per-link transform), `SpatialJacobian` (M8, differentiates this chain),
`ProductOfExponentials` (M15, the screw-theory alternative that skips DH bookkeeping).

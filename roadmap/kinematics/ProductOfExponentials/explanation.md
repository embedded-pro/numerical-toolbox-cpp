# Product of Exponentials — Overview

## What it is
A forward-kinematics formula built from screw theory: `T = e^{[S₁]θ₁}···e^{[Sₙ]θₙ}·M`. Each joint is a
**screw axis** `Sᵢ` fixed in the base frame; turning the joint by `θᵢ` applies the matrix exponential of
that screw, and `M` is the tool's pose when every joint is at zero.

## Why it matters (embedded)
PoE describes an arm with a single home pose `M` and one screw axis per joint — all in the *base*
frame — instead of a chain of intermediate DH frames. There is nothing to line up between links, so it
is less error-prone to author, and the same screw axes give the Jacobian almost for free. For code that
already has an `SE(3)` type, it is the most direct FK you can write.

## How it works (intuition)
A screw axis packages "rotate about this line while sliding along it" into one 6-vector. Its matrix
exponential is the finite rigid motion produced by riding that screw for a parameter `θ` — the
rigid-body version of Rodrigues' rotation formula. Forward kinematics is then just: start at the home
pose, and for each joint multiply in the motion its screw produces. Because every axis is written in
the fixed base frame, you never rebuild intermediate frames; you only compose `SE(3)` exponentials.

## Key parameters
- **screw axes `Sᵢ = (ωᵢ; vᵢ)`** — one per joint, expressed in the base frame.
- **home configuration `M`** — the tool pose at `q = 0`.
- **space vs body form** — whether the screws are read in the fixed or the tool frame.

## Reference
K. M. Lynch, F. C. Park, *Modern Robotics* (2017), Ch. 4 (forward kinematics via the product of
exponentials).

## See also
`SE3Transform` (M6, the `Exp` and adjoint), `MatrixExponential` (item 29, the `se(3)→SE(3)` map),
`DenavitHartenberg` (M7, the frame-based alternative), `SpatialJacobian` (M8).

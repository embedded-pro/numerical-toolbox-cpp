# Continuum Kinematics — Overview

## What it is
Kinematics for arms that bend continuously instead of pivoting at discrete joints — tendon-driven,
pneumatic, or "soft" robots shaped like an elephant's trunk. The standard model treats each section as
a **circular arc of constant curvature**, described by three numbers: how sharply it bends, in which
plane, and over what length.

## Why it matters (embedded)
Continuum robots reach into cluttered, delicate spaces — inside the body for surgery, around obstacles
for inspection — where a rigid arm cannot go. The constant-curvature model reduces an infinite-DOF
flexible body to a handful of arc parameters per section, small and cheap enough to run the forward
kinematics on the embedded controller driving the tendons or air chambers.

## How it works (intuition)
Each section is assumed to bow into a perfect circular arc. Three parameters pin it down: the curvature
`κ` (how tight the arc is), the plane angle `φ` (which way it bends), and the arc length `s` (how far
along). From these, the section's tip pose is a closed-form point on that circle, with the frame
rotated by the total bend angle `θ = κ·s`. Chaining the section transforms — exactly like multiplying
joint transforms on a rigid arm — gives the whole robot's shape. The one delicate spot is a nearly
straight section: the formulas divide by `κ`, so as the arc flattens you must switch to a straight-line
limit to avoid dividing by zero. This "robot-independent" arc mapping is kept separate from the
"robot-specific" step that converts actual tendon pulls or chamber pressures into `(κ, φ, s)`.

## Key parameters
- **curvature `κ`** — inverse bend radius of the section.
- **bending-plane angle `φ`** — the direction the section curves (undefined when straight).
- **arc length `s`** — how long the section is.

## Reference
R. J. Webster III, B. A. Jones, "Design and Kinematic Modeling of Constant Curvature Continuum Robots:
A Review," *Int. J. Robotics Research*, 29(13), 2010.

## See also
`SE3Transform` (M6, per-section transform), `Quaternion` (item 18, orientation blending),
`PoseInverseKinematics` (M13, the iterative multi-section inverse).

# Mobile-Manipulator Kinematics — Overview

## What it is
Kinematics for an arm mounted on a driving base — a robot that can both reach and roam. It combines the
arm's Jacobian with the base's motion into one map from "wheel commands plus joint rates" to
end-effector velocity, while respecting that a wheeled base cannot move sideways.

## Why it matters (embedded)
Mobile manipulators — warehouse robots, service robots, rovers with arms — must coordinate base and arm
as a single system, not two. A unified Jacobian lets one controller command the whole platform, and the
extra freedom of "drive there *and* reach" is what gives these robots their large effective workspace.
Doing it on-board in real time needs a compact, constraint-aware model.

## How it works (intuition)
The tool's velocity is the sum of two contributions: what the arm does on top of a stationary base, and
what the base does while carrying a frozen arm. The arm part is its ordinary Jacobian. The base part
depends on the lever arm from the base to the tool — driving forward translates the tool, spinning the
base swings it around, and the farther out the tool, the more a base rotation moves it. The catch is the
**nonholonomic** constraint: a differential-drive base has three planar coordinates but only two
controls (drive speed and turn rate), because it physically cannot slip sideways. A small matrix `S(φ)`
encodes that, mapping the two real controls into allowed base motion. Stacking the constrained base
block beside the arm Jacobian gives the combined map; because base-plus-arm usually over-provides DOF,
null-space resolution decides how much each should contribute.

## Key parameters
- **base pose `(x, y, φ)`** and **drive controls `(v, ω)`** — the constrained base freedom.
- **arm configuration `q`** — the joint angles feeding the arm Jacobian.
- **base-to-arm mount** — the fixed transform placing the arm on the base.

## Reference
Y. Yamamoto, X. Yun, "Coordinating Locomotion and Manipulation of a Mobile Manipulator," *IEEE Trans.
Automatic Control*, 39(6), 1994.

## See also
`SpatialJacobian` (M8, the arm block), `RedundancyResolution` (M14, splitting base vs arm),
`SE3Transform` (M6, base pose composition).

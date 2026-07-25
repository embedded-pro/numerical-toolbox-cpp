# Quaternion — Overview

## What it is
A four-component hypercomplex number `w + xi + yj + zk` that, constrained to unit norm, encodes a
3D rotation. Quaternions compose rotations by multiplication and rotate vectors by conjugation —
without the singularities that plague Euler angles.

## Why it matters (embedded)
Attitude tracking on drones, robots, and wearables needs a rotation representation that is compact
(4 words vs a 9-word matrix), cheap to compose, and **free of gimbal lock**. A unit quaternion is
the standard state carried by every AHRS filter, and its components live in [-1, 1].

## How it works (intuition)
A rotation by angle `θ` about a unit axis `n̂` is stored as `(cos θ/2, n̂·sin θ/2)`. Multiplying two
quaternions composes their rotations (the Hamilton product). Rotating a vector is `q·(0,v)·q⁻¹`,
which simplifies to a pair of cross products. Interpolating between two orientations along the
shortest constant-speed arc is **SLERP** — spherical linear interpolation on the unit 4-sphere.

## Key parameters
- **Unit-norm constraint** — only unit quaternions rotate; renormalize to fight drift.
- **SLERP parameter `t ∈ [0, 1]`** — fraction along the arc between two orientations.
- **Euler convention** — the roll-pitch-yaw order (ZYX here) for interoperability.

## Reference
J. B. Kuipers, *Quaternions and Rotation Sequences* (1999); K. Shoemake, "Animating rotation with
quaternion curves," *SIGGRAPH*, 1985 (SLERP).

## See also
`Geometry3D` (`RotationAboutAxis`, `CrossProduct`), `AhrsMadgwickMahony` (item 33, the main
consumer), `Cordic` (shift-add trig for the axis-angle conversions).

# Generic (Revolute / Prismatic) Joint Link — Implementation Pseudocode

> Roadmap ref: #M1 (Tier 1) · Target: `numerical/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
enum class JointType : uint8_t { Revolute, Prismatic }

template<typename T>              # static_assert(std::is_floating_point_v<T>); instantiated for float
struct GenericJointLink:
    JointType                 type            # revolute (rotate) or prismatic (slide)
    T                         mass
    math::SquareMatrix<T, 3>  inertia         # inertia tensor at CoM, link frame
    math::Vector<T, 3>        axis            # unit joint axis in link frame
    math::Vector<T, 3>        parentToJoint   # fixed joint origin in parent frame
    math::Vector<T, 3>        jointToCoM      # CoM position in link frame
```

## Interface

```
# Relative transform produced by the joint variable q:
(Matrix3 R, Vector3 p) JointTransform(T q) const            # hot path

# Motion-subspace (screw) split — exactly one is the axis, the other is zero:
Vector3 AngularAxis() const     # axis if Revolute else 0
Vector3 LinearAxis()  const     # axis if Prismatic else 0

# Adapter so existing revolute-only code keeps compiling:
static GenericJointLink FromRevolute(const RevoluteJointLink<T>& link)
```

## Algorithm (pseudocode)

```
function JointTransform(q):                        # OPTIMIZE_FOR_SPEED
    if type == Revolute:
        R = math::RotationAboutAxis(axis, q)       # rotate by angle q about axis
        p = parentToJoint                          # origin fixed
    else:  # Prismatic
        R = Identity3                              # no rotation
        p = parentToJoint + axis * q               # slide distance q along axis
    return (R, p)

function AngularAxis():  return (type == Revolute)  ? axis : 0
function LinearAxis():   return (type == Prismatic) ? axis : 0
```

## Complexity & memory

- `JointTransform`: `O(1)` — one axis-angle rotation (revolute) or one scaled add (prismatic).
- Memory: one enum byte + the existing inertial fields; no growth over `RevoluteJointLink`.

## Numerical / embedded notes

- The `(AngularAxis, LinearAxis)` pair *is* the motion-subspace `S`: FK multiplies `JointTransform`
  down the chain; the 6×N Jacobian (M8) fills column `i` from these two vectors; RNEA injects
  `axis·q̇` into the angular channel (revolute) or the linear channel (prismatic).
- Keep `axis` **unit-normalized** at construction; a non-unit axis silently rescales both the
  rotation angle and the slide distance.
- `RotationAboutAxis` (Rodrigues form) avoids gimbal issues and is branch-light — good for the FK loop.
- A `Prismatic` joint contributes no rotation, so its inertia only shifts via the parallel-axis
  term along `axis·q`; revolute links keep the existing propagation unchanged.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/dynamics/GenericJointLink.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `JointTransform`, and
  `extern template struct GenericJointLink<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/dynamics/GenericJointLink.cpp` →
  `template struct GenericJointLink<float>;`
- Test: `numerical/dynamics/test/TestGenericJointLink.cpp`
- Doc: `doc/dynamics/GenericJointLink.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestGenericJointLink.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".

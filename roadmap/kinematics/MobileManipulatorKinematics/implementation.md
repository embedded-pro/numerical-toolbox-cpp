# Mobile-Manipulator Kinematics (Nonholonomic Base) — Implementation Pseudocode

> Roadmap ref: #M24 (Tier 4) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                            # static_assert(std::is_floating_point_v<T>); instantiated for float
struct BaseState:                               # differential-drive base pose
    T x, y, phi                                 # planar position + heading

template<typename T, std::size_t NumArm>
class MobileManipulatorKinematics:
    SpatialJacobian<T, NumArm>  armJac
    SE3<T>                      baseToArm        # arm mount on the base
    T                           wheelBase        # for the drive model
```

## Interface

```
MobileManipulatorKinematics(armJac, SE3<T> baseToArm, T wheelBase)
Matrix<T, 6, 2 + NumArm>  Compute(BaseState<T> base, JointVector q)    # combined J; hot path
SE3<T>                    EndEffectorPose(BaseState<T> base, JointVector q)
Matrix<T, 3, 2>           BaseConstraint(BaseState<T> base)            # S(φ): (v,ω) → q̇_base
```

## Algorithm (pseudocode)

```
function BaseConstraint(base):                  # nonholonomic: no side-slip
    # Pfaffian constraint  [-sinφ, cosφ, 0]·q̇_base = 0  ⇒ parameterize by (v, ω)
    return [[ cosφ, 0 ],
            [ sinφ, 0 ],
            [   0,  1 ]]                          # q̇_base = S(φ)·(v, ω)

function Compute(base, q):                       # OPTIMIZE_FOR_SPEED
    # end-effector velocity = base contribution + arm contribution
    J_arm  = armJac.Compute(q)                   # 6×NumArm, in the base frame
    r      = EndEffectorPose(base, q).p - basePosition   # lever arm base → tool
    J_base = [[ I₃ , -SkewSymmetric(r) ],         # planar base twist → tool twist
              [ 0  ,        ẑ          ]]          # keep the (v, ω) columns only
    J_base_reduced = J_base * lift(S(φ))          # apply nonholonomic S(φ) ⇒ 6×2
    return [ J_base_reduced | J_arm ]             # 6 × (2 + NumArm)

function EndEffectorPose(base, q):
    T_base = SE3(Rotz(base.phi), (base.x, base.y, 0))
    return T_base * baseToArm * armForward(q)
```

## Complexity & memory

- `Compute`: `O(NumArm)` — arm Jacobian plus a fixed base block and one `S(φ)` multiply.
- Memory: one `6 × (2 + NumArm)` combined Jacobian; bounded, stack-allocated.

## Numerical / embedded notes

- The **nonholonomic constraint** (a differential-drive base cannot slide sideways) drops the base from
  3 planar DOF to 2 controls `(v, ω)`; `S(φ)` enforces it — never command lateral base velocity directly.
- Base and arm together are **redundant** for a 6-DOF task — resolve the split with null-space
  projection (M14), e.g. prefer arm motion for fine moves and base motion for gross reach.
- Compute the tool lever arm `r` in the **base frame** so the `[r]×` block and the arm Jacobian share
  one convention; reuse `SkewSymmetric` (Geometry3D) and `SpatialJacobian` (M8).
- Watch coupling: base rotation `ω` moves a far-out tool a lot (`|r|` large) — scale the two blocks so
  the solver does not over-use the base.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/MobileManipulatorKinematics.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class MobileManipulatorKinematics<float, NumArm>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/MobileManipulatorKinematics.cpp` → `template class MobileManipulatorKinematics<float, NumArm>;`
- Test: `numerical/kinematics/test/TestMobileManipulatorKinematics.cpp`
- Doc: `doc/kinematics/MobileManipulatorKinematics.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestMobileManipulatorKinematics.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

# Hybrid Position/Force Control — Implementation Pseudocode

> Roadmap ref: #M19 (Tier 4) · Target: `numerical/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof, std::size_t TaskDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class HybridPositionForceControl:
    const dynamics::EulerLagrangeDynamics<T, Dof>& model    # g(q), C(q,q̇)q̇
    const kinematics::SpatialJacobian<T, Dof>&    jacobian   # 6×Dof, injected (#M8)
    math::SquareMatrix<T, TaskDim> S         # selection: 1 = motion axis, 0 = force axis
    math::SquareMatrix<T, TaskDim> Kp, Kd    # motion-subspace PD gains
    math::SquareMatrix<T, TaskDim> Kf, Ki    # force-subspace P / I gains
    math::Vector<T, TaskDim> forceIntegral   # accumulated force error (STATE)
    T dt
```

## Interface

```
HybridPositionForceControl(const EulerLagrangeDynamics& model, const SpatialJacobian& jacobian,
                           const SquareMatrix& S,  const SquareMatrix& Kp, const SquareMatrix& Kd,
                           const SquareMatrix& Kf, const SquareMatrix& Ki, T dt)

Vector<T,Dof> ComputeTorque(const StateVector& q,   const StateVector& qDot,
                            const TaskVector& x,     const TaskVector& xd,
                            const TaskVector& xdDot, const TaskVector& fMeasured,
                            const TaskVector& fd)              # hot path
void Reset()                                                       # clears force integral
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, x, xd, xdDot, fMeasured, fd):   # OPTIMIZE_FOR_SPEED
    J     = jacobian.Compute(q)
    xDot  = J * qDot
    # --- motion subspace (S selects position-controlled axes) ---
    Fmotion = Kp*(xd - x) + Kd*(xdDot - xDot)
    # --- force subspace (I−S selects force-controlled axes), PI on force error ---
    eF            = fd - fMeasured
    forceIntegral = forceIntegral + eF * dt
    Fforce        = fd + Kf*eF + Ki*forceIntegral
    # --- complementary partition: an axis is motion- XOR force-controlled ---
    F = S*Fmotion + (Identity(TaskDim) - S)*Fforce
    # map task wrench to joint torque; cancel arm gravity + Coriolis:
    return transpose(J)*F + model.ComputeCoriolisTerms(q, qDot) + model.ComputeGravityTerms(q)
```

## Complexity & memory

- Time: `O(TaskDim·Dof)` for `J·qDot` and `Jᵀ·F`; model terms `O(Dof)`–`O(Dof²)`.
- Memory: `O(TaskDim²)` gains + `O(TaskDim)` integral state; no heap.

## Numerical / embedded notes

- `S` is a **complementary orthogonal projector**: `S² = S` and `S·(I−S) = 0`, so the motion and
  force loops act on disjoint task axes and never fight (Raibert–Craig).
- Axes are expressed in a **constraint frame** aligned with the contact surface — e.g. peg-in-hole:
  normal = force-controlled, insertion/tangential = motion-controlled.
- The force loop is **PI** (not PD): integral action drives steady-state force error to zero on a
  stiff environment. Add anti-windup / `Reset()` on contact loss to stop integral run-off.
- `Jᵀ` mapping only (no inverse) ⇒ passes through kinematic singularities safely, like impedance.
- Reuses the `Jᵀ` wrench map of `ImpedanceControl` (#M17) and the spatial Jacobian (#M8); the added
  force loop is what makes it *hybrid*.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/manipulator/HybridPositionForceControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class HybridPositionForceControl<float, Dof, TaskDim>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/manipulator/HybridPositionForceControl.cpp` →
  `template class HybridPositionForceControl<float, Dof, TaskDim>;`
- Test: `numerical/controllers/manipulator/test/TestHybridPositionForceControl.cpp`
- Doc: `doc/controllers/manipulator/HybridPositionForceControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestHybridPositionForceControl.cpp` → the `_test` target.
- New module: create `numerical/controllers/manipulator/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/controllers/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

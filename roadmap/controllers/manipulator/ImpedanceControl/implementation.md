# Impedance Control — Implementation Pseudocode

> Roadmap ref: #M17 (Tier 3) · Target: `numerical/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof, std::size_t TaskDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class ImpedanceControl:                           # TaskDim = 6 (pose) or 3 (position-only)
    const dynamics::EulerLagrangeDynamics<T, Dof>& model    # g(q), C(q,q̇)q̇
    const kinematics::SpatialJacobian<T, Dof>&    jacobian   # 6×Dof, injected (#M8)
    math::SquareMatrix<T, TaskDim> Md        # desired end-effector inertia
    math::SquareMatrix<T, TaskDim> Dd        # desired damping
    math::SquareMatrix<T, TaskDim> Kstiff    # desired stiffness
```

## Interface

```
# Dynamics model and Jacobian injected (DIP):
ImpedanceControl(const EulerLagrangeDynamics& model, const SpatialJacobian& jacobian,
                 const SquareMatrix& Md, const SquareMatrix& Dd, const SquareMatrix& Kstiff)

Vector<T,Dof> ComputeTorque(const StateVector& q,   const StateVector& qDot,
                            const TaskVector& x,     const TaskVector& xd,
                            const TaskVector& xdDot, const TaskVector& xdDdot,
                            const TaskVector& fExternal)      # hot path
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, x, xd, xdDot, xdDdot, fExt):    # OPTIMIZE_FOR_SPEED
    J     = jacobian.Compute(q)            # TaskDim×Dof spatial Jacobian
    xDot  = J * qDot                        # measured end-effector twist
    eX    = xd    - x                        # Cartesian pose error
    eXDot = xdDot - xDot                      # Cartesian velocity error
    # desired end-effector wrench rendering  Md·ẍ + Dd·ẋ + Kstiff·x = f_ext :
    F = Md * xdDdot + Dd * eXDot + Kstiff * eX + fExt
    # map task wrench to joint torque (Jᵀ); cancel the arm's own gravity + Coriolis:
    return transpose(J) * F
           + model.ComputeCoriolisTerms(q, qDot)
           + model.ComputeGravityTerms(q)
```

## Complexity & memory

- Time: `O(TaskDim·Dof)` for `J·qDot` and `Jᵀ·F`; model terms `O(Dof)`–`O(Dof²)`.
- Memory: `O(TaskDim²)` for the three impedance matrices; no dynamic state, no heap.

## Numerical / embedded notes

- The `Jᵀ` map is always well-defined (no inverse) ⇒ the controller passes through kinematic
  singularities safely; only *inertia shaping* (`Md` ≠ natural inertia) needs the task inertia
  `Λ = (J M⁻¹ Jᵀ)⁻¹` — see `OperationalSpaceControl`. Leaving `Md` at the natural inertia gives
  the cheap, robust "stiffness control" variant.
- **Admittance** is the dual: measure `fExt`, integrate to a motion command, feed a position loop —
  better on stiff/non-backdrivable robots; impedance is better on backdrivable ones.
- Passivity: with SPD `Dd` and `Kstiff` the rendered port is passive ⇒ stable contact with any
  passive environment.
- Diagonal `Kstiff`/`Dd` are chosen per Cartesian axis (e.g. stiff normal, compliant tangential for
  insertion tasks).
- `fExt` comes from a wrist force/torque sensor; low-pass it and match `Dd` to its bandwidth. Set it
  to zero for pure motion impedance without contact sensing.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/manipulator/ImpedanceControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class ImpedanceControl<float, Dof, TaskDim>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/manipulator/ImpedanceControl.cpp` →
  `template class ImpedanceControl<float, Dof, TaskDim>;`
- Test: `numerical/controllers/manipulator/test/TestImpedanceControl.cpp`
- Doc: `doc/controllers/manipulator/ImpedanceControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestImpedanceControl.cpp` → the `_test` target.
- New module: create `numerical/controllers/manipulator/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/controllers/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

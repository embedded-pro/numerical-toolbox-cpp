# Operational-Space Control — Implementation Pseudocode

> Roadmap ref: #M18 (Tier 4) · Target: `numerical/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof, std::size_t TaskDim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class OperationalSpaceControl:
    const dynamics::EulerLagrangeDynamics<T, Dof>& model    # M(q), C(q,q̇)q̇, g(q)
    const kinematics::SpatialJacobian<T, Dof>&    jacobian   # J (TaskDim×Dof), injected (#M8)
    math::SquareMatrix<T, TaskDim> Kp        # task-space position gain
    math::SquareMatrix<T, TaskDim> Kd        # task-space damping gain
    solvers::GaussianElimination<T, Dof> linearSolver        # for the M⁻¹Jᵀ solve
```

## Interface

```
OperationalSpaceControl(const EulerLagrangeDynamics& model, const SpatialJacobian& jacobian,
                        const SquareMatrix& Kp, const SquareMatrix& Kd)

Vector<T,Dof> ComputeTorque(const StateVector& q,    const StateVector& qDot,
                            const TaskVector& x,      const TaskVector& xd,
                            const TaskVector& xdDot,  const TaskVector& xdDdot,
                            const StateVector& tauSecondary)     # hot path
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, x, xd, xdDot, xdDdot, tauSecondary):   # OPTIMIZE_FOR_SPEED
    J = jacobian.Compute(q)                     # TaskDim×Dof
    M = model.ComputeMassMatrix(q)              # Dof×Dof (SPD)
    # --- task-space (operational-space) inertia  Λ = (J M⁻¹ Jᵀ)⁻¹ ---
    MinvJt = linearSolver.SolveColumns(M, transpose(J))   # solve M·X = Jᵀ (no explicit inverse)
    Lambda = inverse(J * MinvJt)                # TaskDim×TaskDim (small: TaskDim ≤ 6)
    # --- dynamically-consistent inverse and null-space projector ---
    Jbar = MinvJt * Lambda                      # M⁻¹JᵀΛ  (Dof×TaskDim)
    N    = Identity(Dof) - transpose(J) * transpose(Jbar)
    # --- task command wrench ---
    xDot = J * qDot
    aX   = xdDdot + Kd*(xdDot - xDot) + Kp*(xd - x)
    mu   = transpose(Jbar) * model.ComputeCoriolisTerms(q, qDot)   # task-space Coriolis/centrifugal
    p    = transpose(Jbar) * model.ComputeGravityTerms(q)          # task-space gravity
    F    = Lambda * aX + mu + p
    # --- joint torque: task term + dynamically-consistent secondary term ---
    return transpose(J) * F + N * tauSecondary
```

## Complexity & memory

- Time: `O(Dof³)` for the `M`-solve (LU/Gaussian); `O(TaskDim³)` for the `Λ` inverse (small);
  `O(Dof²·TaskDim)` for the projector.
- Memory: `O(Dof²)` working matrices; all stack/static, no heap.

## Numerical / embedded notes

- Solve `M·X = Jᵀ` with `solvers::GaussianElimination` (or LU) rather than forming `M⁻¹` —
  cheaper and better-conditioned.
- `Λ = (J M⁻¹ Jᵀ)⁻¹` blows up at kinematic singularities (`J` loses rank); near `det(J M⁻¹ Jᵀ)→0`
  switch to a **damped** inverse (add `σ²I`).
- The dynamically-consistent null-space `N = I − Jᵀ J̄ᵀ` guarantees `tauSecondary` produces **no**
  task-space acceleration — clean priority stacking for redundant arms.
- The `−Λ J̇ q̇` centrifugal correction is added when the Jacobian derivative is available; omitting
  it costs a small high-speed tracking error.
- `Λ` and `Jbar` are small (`TaskDim`-wide) — cache them when the task dimension ≪ `Dof`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/manipulator/OperationalSpaceControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class OperationalSpaceControl<float, Dof, TaskDim>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/manipulator/OperationalSpaceControl.cpp` →
  `template class OperationalSpaceControl<float, Dof, TaskDim>;`
- Test: `numerical/controllers/manipulator/test/TestOperationalSpaceControl.cpp`
- Doc: `doc/controllers/manipulator/OperationalSpaceControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestOperationalSpaceControl.cpp` → the `_test` target.
- New module: create `numerical/controllers/manipulator/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/controllers/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

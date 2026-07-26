# Computed-Torque Control — Implementation Pseudocode

> Roadmap ref: #M12 (Tier 3) · Target: `numerical/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof>    # static_assert(std::is_floating_point_v<T>); instantiated for float
class ComputedTorqueControl:
    const dynamics::EulerLagrangeDynamics<T, Dof>& model   # M(q), C(q,q̇)q̇, g(q)
    math::SquareMatrix<T, Dof> Kp        # position-error gain
    math::SquareMatrix<T, Dof> Kd        # velocity-error gain
```

## Interface

```
# Full dynamics model injected (DIP); gains chosen for the resulting double integrator:
ComputedTorqueControl(const EulerLagrangeDynamics<T,Dof>& model,
                      const SquareMatrix& Kp, const SquareMatrix& Kd)

Vector<T,Dof> ComputeTorque(const StateVector& q,      const StateVector& qDot,
                            const StateVector& qd,     const StateVector& qdDot,
                            const StateVector& qdDdot)          # hot path
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, qd, qdDot, qdDdot):     # OPTIMIZE_FOR_SPEED
    e    = qd    - q
    eDot = qdDot - qDot
    # inner-loop joint-space acceleration command (feedforward + PD):
    aq   = qdDdot + Kd * eDot + Kp * e
    # inverse-dynamics torque that realises aq exactly:
    #   τ = M(q)·aq + C(q,q̇)q̇ + g(q)
    M    = model.ComputeMassMatrix(q)
    Cqd  = model.ComputeCoriolisTerms(q, qDot)
    g    = model.ComputeGravityTerms(q)
    return M * aq + Cqd + g
```

## Complexity & memory

- Time: `O(Dof²)` for `M·aq`; model evaluation `O(Dof)`–`O(Dof²)` (RNEA inverse dynamics is `O(Dof)`).
- Memory: `O(Dof²)` for the two gains; no dynamic state, no heap.

## Numerical / embedded notes

- Substituting `τ` into `M q̈ + Cq̇ + g = τ` gives the **decoupled** linear error dynamics
  `ë + Kd·ė + Kp·e = 0` — every joint becomes an independent, tunable second-order system.
- The law **multiplies** by `M(q)` (SPD) — it never inverts it, so the hot path stays
  well-conditioned (unlike forward dynamics).
- RNEA supplies `M`, `C·q̇`, `g` without forming `C` explicitly; the injected model wraps that
  detail (DIP), so the controller is agnostic to how the terms are produced.
- Model error leaves a residual (`ë + Kd·ė + Kp·e = M⁻¹Δ`); pair with an integral, robust
  (sliding-mode), or adaptive (`SlotineLiAdaptiveControl`) term to reject it.
- Choose `Kd = 2√Kp` per channel for a critically-damped response.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/manipulator/ComputedTorqueControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class ComputedTorqueControl<float, Dof>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/manipulator/ComputedTorqueControl.cpp` →
  `template class ComputedTorqueControl<float, Dof>;`
- Test: `numerical/controllers/manipulator/test/TestComputedTorqueControl.cpp`
- Doc: `doc/controllers/manipulator/ComputedTorqueControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestComputedTorqueControl.cpp` → the `_test` target.
- New module: create `numerical/controllers/manipulator/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/controllers/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

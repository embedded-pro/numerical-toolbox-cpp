# Feedback Linearization (Computed Torque) — Implementation Pseudocode

> Roadmap ref: #40 (Tier 4) · Target: `numerical/nonlinear_control` · Namespace `nonlinear_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class FeedbackLinearization:
    const dynamics::EulerLagrangeDynamics<T, Dof>& model   # injected M(q), C(q,q̇)q̇, g(q)
    math::SquareMatrix<T, Dof> Kp        # outer proportional gain
    math::SquareMatrix<T, Dof> Kd        # outer derivative gain
```

## Interface

```
# Nonlinear model injected (DIP); outer linear gains chosen for the double-integrator:
FeedbackLinearization(const EulerLagrangeDynamics<T,Dof>& model,
                      const SquareMatrix& Kp, const SquareMatrix& Kd)

Vector<T,Dof> ComputeTorque(const StateVector& q,    const StateVector& qDot,
                            const StateVector& qd,   const StateVector& qdDot,
                            const StateVector& qdDdot)          # hot path
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, qd, qdDot, qdDdot):        # OPTIMIZE_FOR_SPEED
    # --- outer loop: linear control on the linearized plant ÿ = q̈ = v ---
    e    = qd    - q
    eDot = qdDot - qDot
    v    = qdDdot + Kd * eDot + Kp * e         # virtual acceleration command

    # --- inner loop: cancel the known nonlinearities (input-state form) ---
    #   τ = M(q)·v + C(q,q̇)q̇ + g(q)  ⇒  q̈ = v exactly
    M   = model.ComputeMassMatrix(q)
    Cqd = model.ComputeCoriolisTerms(q, qDot)
    g   = model.ComputeGravityTerms(q)
    return M * v + Cqd + g

# General SISO input-output form (relative degree r), for non-mechanical plants:
#   y^(r) = L_f^r h(x) + L_g L_f^(r-1) h(x) · u
#   u = ( v - L_f^r h(x) ) / ( L_g L_f^(r-1) h(x) )      # requires L_g L_f^(r-1) h ≠ 0
```

## Complexity & memory

- `ComputeTorque`: `O(Dof²)` for `M·v`; model evaluation is `O(Dof)`–`O(Dof²)` (RNEA-style).
- Memory: `O(Dof²)` for the two gains; no dynamic state — all static, no heap.

## Numerical / embedded notes

- Computed-torque **multiplies** by `M(q)` — it never inverts it, so no ill-conditioned solve on
  the hot path (unlike forward dynamics). `M(q)` is SPD for mechanical systems.
- Cancellation is only as good as the model: parameter mismatch leaves a residual nonlinearity —
  pair with a robust (sliding-mode) or adaptive (MRAC) outer term to mop up the error.
- The general input-output form loses well-posedness where `L_g L_f^(r-1) h → 0` (a singularity);
  keep the operating region away from it, and watch for unstable internal dynamics (zero dynamics).
- Choose `Kp`, `Kd` for a critically-damped double integrator (`Kd = 2√Kp`) per channel.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/nonlinear_control/FeedbackLinearization.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class FeedbackLinearization<float, Dof>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/nonlinear_control/FeedbackLinearization.cpp` →
  `template class FeedbackLinearization<float, Dof>;`
- Test: `numerical/nonlinear_control/test/TestFeedbackLinearization.cpp`
- Doc: `doc/nonlinear_control/FeedbackLinearization.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestFeedbackLinearization.cpp` → the `_test` target.
- New module: create `numerical/nonlinear_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/nonlinear_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

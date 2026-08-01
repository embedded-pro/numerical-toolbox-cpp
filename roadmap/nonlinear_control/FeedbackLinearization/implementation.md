# Feedback Linearization — Implementation Pseudocode

> Roadmap ref: #40 (Tier 4) · Target: `numerical/nonlinear_control` · Namespace `nonlinear_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
# Injected abstract model of a control-affine plant y^(r) = a(x) + B(x)·u :
template<typename T, std::size_t Dim>
class ControlAffineModel:                       # pure-virtual interface (DIP)
    virtual ~ControlAffineModel() = default
    virtual SquareMatrix<T,Dim> DecouplingMatrix(const StateVector& x) const  # B(x)
    virtual Vector<T,Dim>       DriftTerm(const StateVector& x) const          # a(x) to cancel

template<typename T, std::size_t Dim>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class FeedbackLinearization:
    const ControlAffineModel<T, Dim>& model   # supplies B(x), a(x)
    math::SquareMatrix<T, Dim> Kp             # outer proportional gain
    math::SquareMatrix<T, Dim> Kd             # outer derivative gain
```

The mechanical *computed-torque* instance sets `B(x) = M(q)` and `a(x) = C(q,q̇)q̇ + g(q)`; that
manipulator model lives in robotics-toolbox-cpp and is injected here through `ControlAffineModel`.

## Interface

```
# Nonlinear model injected (DIP); outer linear gains chosen for the integrator chain:
FeedbackLinearization(const ControlAffineModel<T,Dim>& model,
                      const SquareMatrix& Kp, const SquareMatrix& Kd)

Vector<T,Dim> ComputeInput(const StateVector& x,   const StateVector& xDot,
                           const StateVector& yd,  const StateVector& ydDot,
                           const StateVector& ydDdot)          # hot path
```

## Algorithm (pseudocode)

```
function ComputeInput(x, xDot, yd, ydDot, ydDdot):        # OPTIMIZE_FOR_SPEED
    # --- outer loop: linear control on the linearized plant ÿ = v ---
    e    = yd    - x
    eDot = ydDot - xDot
    v    = ydDdot + Kd * eDot + Kp * e         # virtual input command

    # --- inner loop: cancel the known drift (input-state form) ---
    #   u = B(x)·v + a(x)  ⇒  ÿ = v exactly
    B = model.DecouplingMatrix(x)
    a = model.DriftTerm(x)
    return B * v + a

# General SISO input-output form (relative degree r), for non-mechanical plants:
#   y^(r) = L_f^r h(x) + L_g L_f^(r-1) h(x) · u
#   u = ( v - L_f^r h(x) ) / ( L_g L_f^(r-1) h(x) )      # requires L_g L_f^(r-1) h ≠ 0
```

## Complexity & memory

- `ComputeInput`: `O(Dim²)` for `B·v`; model evaluation is `O(Dim)`–`O(Dim²)`.
- Memory: `O(Dim²)` for the two gains; no dynamic state — all static, no heap.

## Numerical / embedded notes

- The input-state form **multiplies** by `B(x)` — it never inverts it, so no ill-conditioned solve
  on the hot path (unlike forward dynamics). For mechanical plants `B(x) = M(q)` is SPD.
- Cancellation is only as good as the model: parameter mismatch leaves a residual nonlinearity —
  pair with a robust (sliding-mode) or adaptive (MRAC) outer term to mop up the error.
- The general input-output form loses well-posedness where `L_g L_f^(r-1) h → 0` (a singularity);
  keep the operating region away from it, and watch for unstable internal dynamics (zero dynamics).
- Choose `Kp`, `Kd` for a critically-damped integrator chain (`Kd = 2√Kp`) per channel.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/nonlinear_control/FeedbackLinearization.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeInput`, and
  `extern template class FeedbackLinearization<float, Dim>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/nonlinear_control/FeedbackLinearization.cpp` →
  `template class FeedbackLinearization<float, Dim>;`
- Test: `numerical/nonlinear_control/test/TestFeedbackLinearization.cpp`
- Doc: `doc/nonlinear_control/FeedbackLinearization.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestFeedbackLinearization.cpp` → the `_test` target.
- New module: create `numerical/nonlinear_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/nonlinear_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

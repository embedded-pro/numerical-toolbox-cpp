# Levenberg–Marquardt (damped Gauss–Newton with retraction) — Implementation Pseudocode

> Roadmap ref: #64 (Tier 3) · Target: `numerical/optimization` · Namespace `optimization` · Type: `float` (templated on `T`, instantiated for `float` only)

Bounded-iteration solver for small nonlinear least-squares problems `min ½‖r(x)‖²` whose state may live
on a manifold (an angle, a rotation, a pose) or inside bounds (joint limits). The problem supplies the
residual, its Jacobian with respect to a local increment `δ`, and a **retraction** `x ⊕ δ`; the solver
supplies the damped Gauss–Newton step. Consumers: pose inverse kinematics and parallel-robot forward
kinematics (robotics-toolbox-cpp), hand-eye and sensor calibration, scan matching.

## Data structures

```cpp
template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
class LeastSquaresProblem:                     # pure interface; static_assert(std::is_floating_point_v<T>)
    virtual ~LeastSquaresProblem() = default
    virtual math::Vector<T, Residuals> Residual(const State& x) const = 0
    virtual math::Matrix<T, Residuals, Parameters> Jacobian(const State& x) const = 0   # ∂r(x ⊕ δ)/∂δ at δ = 0
    virtual State Retract(const State& x, const math::Vector<T, Parameters>& delta) const = 0
        # x ⊕ δ: plain addition, addition + clamping to bounds, or a manifold update (e.g. R ← Exp(δ)·R)

template<typename T, std::size_t Residuals, std::size_t Parameters>
class DampingPolicy:                           # pure interface
    virtual ~DampingPolicy() = default
    virtual T DampingSquared(const math::Matrix<T, Residuals, Parameters>& jacobian) const = 0   # λ² ≥ 0

template<typename T, std::size_t Residuals, std::size_t Parameters>
class ConstantDamping : public DampingPolicy<T, Residuals, Parameters>:
    T lambdaSquared                            # returns it unconditionally

template<typename T>
struct LevenbergMarquardtConfig:
    T           tolerance                      # on ‖r‖
    T           maxStep                        # cap on ‖δ‖; +∞ disables
    std::size_t maxIterations

template<typename T, typename State>
struct LevenbergMarquardtResult:
    State       x
    T           residualNorm                   # ‖r(x)‖ at the returned x
    std::size_t iterations
    bool        converged

template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>   # instantiated for float
class LevenbergMarquardt:
    const LeastSquaresProblem<T, State, Residuals, Parameters>& problem   # injected
    const DampingPolicy<T, Residuals, Parameters>&              damping   # injected
    LevenbergMarquardtConfig<T>                                 config
```

## Interface

```cpp
LevenbergMarquardt(const LeastSquaresProblem<...>& problem, const DampingPolicy<...>& damping,
                   const LevenbergMarquardtConfig<T>& config)
LevenbergMarquardtResult<T, State> Solve(const State& initial) const      # hot path
```

## Algorithm (pseudocode)

```text
function Solve(x0):                            # OPTIMIZE_FOR_SPEED
    x = x0
    for iter in 0..maxIterations−1:
        r = problem.Residual(x)
        if ‖r‖ < tolerance: return { x, ‖r‖, iter, true }
        J  = problem.Jacobian(x)
        λ² = damping.DampingSquared(J)
        if Residuals ≤ Parameters:                             # wide or square: solve in residual space
            lu = LuDecomposition<T, Residuals>(J·Jᵀ + λ²·I)
            if !lu: return { x, ‖r‖, iter, false }             # singular undamped system
            δ = −Jᵀ·lu.Solve(r)                                # = −Jᵀ(JJᵀ + λ²I)⁻¹r, minimum-norm step
        else:                                                  # tall: solve in parameter space
            lu = LuDecomposition<T, Parameters>(Jᵀ·J + λ²·I)
            if !lu: return { x, ‖r‖, iter, false }
            δ = −lu.Solve(Jᵀ·r)
        if ‖δ‖ > maxStep: δ = δ·maxStep/‖δ‖
        x = problem.Retract(x, δ)
    rFinal = ‖problem.Residual(x)‖                              # recomputed at the returned x
    return { x, rFinal, maxIterations, rFinal < tolerance }
```

## Complexity & memory

- Per iteration: one `Residual` + one `Jacobian` (problem cost), `O(Residuals·Parameters·m)` for the
  normal matrix and an `O(m³)` LU, `m = min(Residuals, Parameters)`.
- Memory: one Jacobian, one `m×m` system; stack only, no heap. Deterministic upper bound:
  `maxIterations + 1` residual evaluations.

## Numerical / embedded notes

- **Damping does not bias the answer:** at a fixed point with `J` of full rank the step is zero only when
  `r = 0` (wide) or `Jᵀr = 0` (tall); damping slows convergence (linear instead of quadratic) but does not
  move the solution. A non-zero final residual means an unreachable target, a singularity or an active
  bound in `Retract`.
- **Policies:** `ConstantDamping` is plain Levenberg–Marquardt with fixed `λ` (`λ = 0`: Gauss–Newton).
  A problem-aware policy — e.g. robotics-toolbox-cpp's manipulability-based IK damping, zero away from
  singularities — plugs in through the same interface. Gain-ratio (Nielsen) adaptation, which needs a
  trial evaluation and accept/reject, is a possible later variant.
- Choosing the smaller normal system keeps the solve `min(R, P)`-sized; use `LuDecomposition`, never an
  explicit inverse. Upstream LU reports singular below a relative pivot of `1e-6`; with `λ = 0` a
  singular `J` therefore ends the solve with `converged = false` instead of dividing by zero.
- `maxStep` bounds the change per iteration (trust-region-like safety for poor initial guesses).
- Warm-start from the previous control cycle for few-iteration convergence.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Headers: `numerical/optimization/LevenbergMarquardt.hpp` (+ `LeastSquaresProblem.hpp`, `DampingPolicy.hpp`,
  pure interfaces) — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Solve`, and
  `extern template class LevenbergMarquardt<float, math::Vector<float, 2>, 2, 2>;` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/optimization/LevenbergMarquardt.cpp` → the same instantiation (and `ConstantDamping<float, 2, 2>`).
- Test: `numerical/optimization/test/TestLevenbergMarquardt.cpp`
- Doc: `doc/optimization/LevenbergMarquardt.md` (per `doc/TEMPLATE.md`) + row in `doc/optimization/README.md`
- CMake: `.hpp` → `target_sources(numerical.optimization PRIVATE ...)` (link `numerical.solver` for LU);
  `.cpp` → `numerical_add_coverage_sources(numerical.optimization ...)`; test → the optimization `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

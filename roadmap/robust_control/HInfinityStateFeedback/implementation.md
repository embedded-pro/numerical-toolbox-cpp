# H∞ State-Feedback Control — Implementation Pseudocode

> Roadmap ref: #46 (Tier 5) · Target: `numerical/robust_control` · Namespace `robust_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class HInfinityStateFeedback:
    math::LinearTimeInvariant<T,StateSize,ControlSize,ErrorSize> plant  # A, B2 (control), C1
    math::Matrix<T, StateSize, DisturbanceSize> B1     # disturbance channel
    math::Matrix<T, ErrorSize,  ControlSize>    D12    # error-vs-control weighting
    math::Matrix<T, ControlSize, StateSize>     K      # state-feedback gain, u = -K·x
    math::Matrix<T, StateSize,   StateSize>     X      # game-Riccati solution
    T gamma                                            # achieved disturbance attenuation
```

## Interface

```
HInfinityStateFeedback(const GeneralizedPlant& plant)     # A,B1,B2,C1,D12

bool Synthesize(T gammaMin, T gammaMax, T tol)             # bisection for smallest feasible γ
InputVector ComputeControl(const StateVector& x) const     # hot path: u = -K·x
const Matrix<T,ControlSize,StateSize>& Gain() const
T Gamma() const
```

## Algorithm (pseudocode)

```
function Synthesize(gLo, gHi, tol):
    # bisection: the smallest γ that still admits a stabilizing PSD Riccati solution
    while gHi - gLo > tol:
        g = (gLo + gHi) / 2
        if RiccatiFeasible(g): gHi = g            # attenuation achievable -> tighten
        else:                  gLo = g            # infeasible -> relax
    gamma = gHi
    (X, K) = SolveGameRiccati(gamma)
    return IsSchurStable(plant.A - plant.B2 * K)

function SolveGameRiccati(g):                     # game-theoretic DARE (GARE)
    # Two antagonistic inputs: control u minimizes, disturbance w maximizes.
    # Stack B = [B2 | B1] with an INDEFINITE input weight  R̃ = diag( I , -g²·I ).
    B = HStack(plant.B2, B1)
    X = DiscreteAlgebraicRiccatiEquation(plant.A, B, C1ᵀ*C1, R̃).Solve()   # reuse DARE
    Kfull = Inverse(R̃ + Bᵀ*X*B) * (Bᵀ*X*plant.A)   # full [u;w] gain
    K = ControlRows(Kfull)                          # keep the control (u) block only
    return (X, K)

function ComputeControl(x):                       # OPTIMIZE_FOR_SPEED
    return K * x * (-1)                            # u = -K·x
```

## Complexity & memory

- `Synthesize`: `O(log((γHi−γLo)/tol))` Riccati solves, each `O(StateSize³)`.
- `ComputeControl`: `O(ControlSize·StateSize)` — one matrix-vector product.
- Memory: `O(StateSize²)` for `X`, `B1`, and the gain — all static, no heap.

## Numerical / embedded notes

- **`γ` is the guaranteed L2 gain** from disturbance `w` to error `z`. Smaller `γ` = more robust but
  harder to solve; bisection converges to the (sub)optimal `γ*`.
- **Feasibility** requires a PSD stabilizing `X` **and** `(−γ²I + B1ᵀX B1) ≺ 0` so the disturbance block
  stays a genuine maximizer; `RiccatiFeasible` checks both before accepting `g`.
- As `γ → ∞` the antagonist vanishes and the solution **collapses to LQR** — H∞ is its robust generalization.
- **Reuse `DiscreteAlgebraicRiccatiEquation`** for the inner solve (items 29 & 31 for the matrix
  algebra); never reimplement the Riccati recursion (DRY).
- Always verify `A − B2·K` is **Schur-stable** after synthesis; reject the design if not.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/robust_control/HInfinityStateFeedback.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeControl`, and
  `extern template class HInfinityStateFeedback<float, StateSize, DisturbanceSize, ControlSize, ErrorSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/robust_control/HInfinityStateFeedback.cpp` →
  `template class HInfinityStateFeedback<float, StateSize, DisturbanceSize, ControlSize, ErrorSize>;`
- Test: `numerical/robust_control/test/TestHInfinityStateFeedback.cpp`
- Doc: `doc/robust_control/HInfinityStateFeedback.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestHInfinityStateFeedback.cpp` → the `_test` target.
- New module: create `numerical/robust_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/robust_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

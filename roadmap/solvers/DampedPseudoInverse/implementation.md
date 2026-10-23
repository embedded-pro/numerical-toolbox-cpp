# Damped Pseudo-Inverse and Null-Space Projector — Implementation Pseudocode

> Roadmap ref: #62 (Tier 2) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

Singularity-robust solution of `A·x = b` for any `Rows × Cols` matrix, plus the exact projector onto the
null space of `A`, both from one SVD. The damped (Tikhonov) inverse `A⁺_λ` keeps `x` bounded when `A`
loses rank; the projector `P = I − A⁺A` lets a secondary objective move `x` without changing `A·x`.
Consumers: redundant-manipulator resolution and damped IK (robotics-toolbox-cpp), control allocation
for over-actuated drones and vessels, regularized calibration.

## Data structures

```cpp
template<typename T, std::size_t Rows, std::size_t Cols>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class DampedPseudoInverse:
    SingularValueDecomposition<T, Rows, Cols> svd          # tall or wide; k = min(Rows, Cols)
    T damping                                              # λ ≥ 0 — damps the solve only
    T rankTolerance                                        # relative: σᵢ > rankTolerance·σ₀ counts toward rank
    std::size_t rank                                       # r, set by Factor
```

## Interface

```cpp
DampedPseudoInverse(T damping, T rankTolerance = 1e-4)
bool Factor(const math::Matrix<T, Rows, Cols>& a)                      # one SVD; false if it does not converge
math::Vector<T, Cols>  Solve(const math::Vector<T, Rows>& b) const     # A⁺_λ·b; hot path
math::Vector<T, Cols>  Project(const math::Vector<T, Cols>& x) const   # (I − A⁺A)·x, undamped; hot path
math::Matrix<T, Cols, Rows> Inverse() const                            # A⁺_λ
math::Matrix<T, Cols, Cols> NullSpaceProjector() const                 # I − A⁺A
std::size_t Rank() const
```

## Algorithm (pseudocode)

```text
function Factor(a):
    if !svd.Decompose(a): return false         # A = U·Σ·Vᵀ;  uᵢ = column i of U (Rows), vᵢ = column i of V (Cols)
    τ = rankTolerance·σ₀;  rank = #{ σᵢ > τ }
    return true

function Gain(σ):                              # damped inverse of one singular value
    λ > 0 ? σ / (σ² + λ²) : (σ > τ ? 1/σ : 0)

function Solve(b):                             # OPTIMIZE_FOR_SPEED — O(Rows·k + Cols·k)
    return Σᵢ Gain(σᵢ)·(uᵢ·b)·vᵢ               # lies in the row space of A ⇒ minimum norm

function Project(x):                           # OPTIMIZE_FOR_SPEED — without forming P, O(Cols·r)
    return x − Σ_{i<r} (vᵢ·x)·vᵢ

function Inverse():            return Σᵢ Gain(σᵢ)·vᵢ·uᵢᵀ
function NullSpaceProjector(): return I_Cols − Σ_{i<r} vᵢ·vᵢᵀ          # = I − A⁺A with A⁺ = svd.PseudoInverse(τ)
```

## Complexity & memory

- `Factor`: one Golub–Kahan SVD (`O(max·k²)`, wide matrices through their transpose).
- `Solve` / `Project`: `O((Rows + Cols)·k)`; `Inverse` / `NullSpaceProjector` (diagnostics) `O(Rows·Cols·k)`
  and `O(Cols²·r)`.
- Memory: SVD factors (`Rows×k`, `Cols×k`, `k`); stack only, no heap.

## Numerical / embedded notes

- **Why the projector is undamped:** with the damped inverse, `A·(I − A⁺_λA) = λ²(AAᵀ + λ²I)⁻¹A ≠ 0`, so
  `I − A⁺_λA` is neither idempotent nor invisible to the primary objective. Built from the thresholded SVD,
  `P` is an exact orthogonal projector: `P² = P`, `Pᵀ = P`, `A·P = Σ_{σᵢ ≤ τ} σᵢuᵢvᵢᵀ` (zero for exact rank
  loss, rounding-level at full rank).
- **Damping bias:** `A·Solve(b) = Σᵢ σᵢ·Gain(σᵢ)·uᵢuᵢᵀ·b`; for `b` in the range of `A`,
  `‖A·Solve(b) − b‖ ≤ λ²/(σ_min² + λ²)·‖b‖`. The damped gain never exceeds `1/(2λ)`.
- Near rank loss the threshold grows the null space: a lost direction is not protected from `Project`'s
  secondary motion — by design.
- Factor once per matrix; `Solve` and `Project` then cost only inner products (no division).
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/solvers/DampedPseudoInverse.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on
  `Solve`/`Project`, and `extern template class DampedPseudoInverse<float, 2, 3>;` / `<float, 6, 7>` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/DampedPseudoInverse.cpp` → the same instantiations.
- Test: `numerical/solvers/test/TestDampedPseudoInverse.cpp`
- Doc: `doc/solvers/DampedPseudoInverse.md` (per `doc/TEMPLATE.md`) + row in `doc/solvers/README.md`
- CMake: `.hpp` → `target_sources(numerical.solver PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.solver ...)`; `TestDampedPseudoInverse.cpp` → `numerical.solvers_test`.
- Depends on: `SingularValueDecomposition` with wide-matrix support.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

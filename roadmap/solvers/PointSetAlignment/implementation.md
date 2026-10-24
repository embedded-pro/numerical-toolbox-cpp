# Point-Set Alignment (Kabsch / Umeyama) — Implementation Pseudocode

> Roadmap ref: #67 (Tier 2) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

Weighted least-squares rigid (or similarity) transform between two corresponding point sets:
find `R ∈ SO(d)`, `t` and optionally `s > 0` minimising `Σ wᵢ ‖qᵢ − (s·R·pᵢ + t)‖²`. This is the inner step
of point-to-point ICP, hand-eye and frame calibration, fiducial and motion-capture registration. In 3-D
it is one 3×3 SVD with Kabsch's reflection fix; in 2-D a closed-form `atan2`.

## Data structures

```cpp
template<typename T, std::size_t Dim>           # static_assert(std::is_floating_point_v<T>); static_assert(Dim == 2 || Dim == 3)
struct Alignment:
    math::SquareMatrix<T, Dim> rotation         # proper: det = +1
    math::Vector<T, Dim>       translation
    T                          scale{ 1 }       # 1 unless estimated
    T                          rms              # weighted residual RMS after alignment

template<typename T, std::size_t Dim>
class PointSetAlignment                         # stateless; static members only
```

## Interface

```cpp
static std::optional<Alignment<T, Dim>> Align(std::span<const math::Vector<T, Dim>> source,
                                              std::span<const math::Vector<T, Dim>> target,
                                              bool estimateScale = false,
                                              std::span<const T> weights = {})     # empty = all 1
```

`target[i]` corresponds to `source[i]`. Returns `nullopt` when the sizes differ, when fewer than `Dim`
points carry positive weight, or when the geometry leaves the rotation undetermined (see below).

## Algorithm (pseudocode)

```text
function Align(p, q, estimateScale, w):          # OPTIMIZE_FOR_SPEED
    W = Σ wᵢ;  μp = Σ wᵢ pᵢ / W;  μq = Σ wᵢ qᵢ / W
    a_i = pᵢ − μp;  b_i = qᵢ − μq
    σp² = Σ wᵢ ‖a_i‖² / W;  σq² = Σ wᵢ ‖b_i‖² / W  # spreads: σp² sets the scale
    Σ   = Σ wᵢ b_i a_iᵀ / W                      # Dim×Dim cross-covariance

    if Dim == 2:                                 # closed form, no SVD, never a reflection
        c = Σ₀₀ + Σ₁₁;  s_ = Σ₁₀ − Σ₀₁           # Σ wᵢ (a·b) and Σ wᵢ (a × b), over W
        if Hypot(c, s_) ≤ ε·σp·σq: return nullopt          # all points coincide
        θ = atan2(s_, c);  R = Rot(θ);  trace = Hypot(c, s_)
    else:
        SVD: Σ = U·diag(d)·Vᵀ                    # solvers::SingularValueDecomposition<T, 3, 3>
        if d₁ ≤ ε·d₀: return nullopt             # collinear (or coincident) points: rotation about the line is free
        E = diag(1, 1, sign(det U · det V))      # Kabsch: never return a reflection
        R = U·E·Vᵀ;  trace = d₀ + d₁ + E₂₂·d₂

    s = estimateScale ? trace / σp² : 1          # Umeyama (1991), eq. 42
    t = μq − s·R·μp
    rms = √( Σ wᵢ ‖qᵢ − (s·R·pᵢ + t)‖² / W )
    return { R, t, s, rms }
```

`ε = 1e-5`. `det U` and `det V` use the explicit 3×3 cofactor formula.

## Complexity & memory

- `O(N·Dim²)` accumulation plus one 3×3 SVD (3-D) or one `atan2` (2-D). One pass for the moments and
  one for the residual.
- Stack only: a few `Dim×Dim` matrices. The point sets stay in the caller's storage. No heap.

## Numerical / embedded notes

- **Reflection fix.** The unconstrained Procrustes solution `U·Vᵀ` can be a reflection (`det = −1`) for
  noisy or near-planar data; `E` flips the weakest axis instead. The test feeds a mirrored point set and
  checks that the result is a proper rotation with a non-zero residual.
- **Degeneracy.** In 3-D a rank-1 cross-covariance (collinear points) leaves the rotation about the line
  free, so `Align` refuses instead of returning an arbitrary answer. Coplanar points are fine: the sign
  fix resolves them. In 2-D only coincident points are degenerate.
- **Centring first.** Accumulating `Σ` on centred coordinates avoids the `float` cancellation of the
  textbook `Σ pqᵀ − N·μp μqᵀ` form when the clouds are far from the origin (scan matching in map
  coordinates).
- Weights let ICP (robotics E5) down-weight or drop outliers without copying the point sets.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/solvers/PointSetAlignment.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Align`, and
  `extern template class PointSetAlignment<float, 2>;` / `<float, 3>` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/PointSetAlignment.cpp` → the same instantiations.
- Test: `numerical/solvers/test/TestPointSetAlignment.cpp`
- Doc: `doc/solvers/PointSetAlignment.md` (per `doc/TEMPLATE.md`) + row in `doc/solvers/README.md`
- CMake: `.hpp` → `target_sources(numerical.solver PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.solver ...)`; test → `numerical.solvers_test`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

# Cubic Spline Interpolation (vector-valued, clamped / natural) — Implementation Pseudocode

> Roadmap ref: #61 (Tier 2) · Target: `numerical/math` · Namespace `math` · Type: `float` (templated on `T`, instantiated for `float` only)

C² piecewise-cubic interpolant through `K` knots `(x_k, y_k)`, with `y_k` a `Dimension`-vector so several
channels that share the abscissa (the joints of a robot, the axes of a CNC path, the channels of a
calibration table) are fitted in one pass. Ends are **clamped** (prescribed slopes) or **natural** (zero
second derivative). Fitting is one tridiagonal solve (#59) whose factorization depends only on the knot
spacing and is shared by all channels.

## Data structures

```cpp
enum class SplineEnd : uint8_t { Clamped, Natural }

template<typename T, std::size_t Dimension>        # static_assert(std::is_floating_point_v<T>)
struct SplineBoundary:
    SplineEnd condition = SplineEnd::Clamped
    math::Vector<T, Dimension> startSlope{}, endSlope{}   # Clamped only; default zero

template<typename T, std::size_t Dimension>
struct SplinePoint:
    math::Vector<T, Dimension> value, firstDerivative, secondDerivative

template<typename T, std::size_t Dimension, std::size_t MaxKnots>   # static_assert(MaxKnots >= 2); instantiated for float
class CubicSpline:
    using Value = math::Vector<T, Dimension>
    std::array<T, MaxKnots>     knots              # x_0 < x_1 < … < x_{K−1}
    std::array<Value, MaxKnots> values             # y_k
    std::array<Value, MaxKnots> secondDerivatives  # M_k = y''(x_k), solved by Fit
    std::array<T, MaxKnots>     inverseSpan        # 1/h_k, h_k = x_{k+1} − x_k
    std::size_t                 count              # K, 2 ≤ K ≤ MaxKnots
```

## Interface

```cpp
bool Fit(const std::array<T, MaxKnots>& knots, const std::array<Value, MaxKnots>& values,
         std::size_t count, const SplineBoundary<T, Dimension>& boundary = {})
                                    # false if count ∉ [2, MaxKnots] or knots not strictly increasing
SplinePoint<T, Dimension> Evaluate(T x) const   # hot path; x clamped to [x_0, x_{K−1}]; precondition: Fit() == true
T First() const;  T Last() const                # x_0, x_{K−1}
```

## Algorithm (pseudocode)

```text
# Per channel j, unknowns M_k = y''(x_k). On interval k (τ = x − x_k ∈ [0, h_k]):
#   y(τ) = y_k + β_k·τ + (M_k/2)·τ² + d_k·τ³
#   β_k  = (y_{k+1} − y_k)/h_k − h_k·(2M_k + M_{k+1})/6        # slope at x_k
#   d_k  = (M_{k+1} − M_k)/(6h_k)
# Value interpolates the knots and the second derivative is shared at knots ⇒ C⁰ and C² by construction.
# Slope continuity at interior knots k = 1..K−2 gives the tridiagonal rows
#   h_{k−1}·M_{k−1} + 2(h_{k−1} + h_k)·M_k + h_k·M_{k+1} = 6·[(y_{k+1} − y_k)/h_k − (y_k − y_{k−1})/h_{k−1}]
# End rows:
#   Clamped:  2h_0·M_0 + h_0·M_1                   = 6·[(y_1 − y_0)/h_0 − s_start]
#             h_{K−2}·M_{K−2} + 2h_{K−2}·M_{K−1}   = 6·[s_end − (y_{K−1} − y_{K−2})/h_{K−2}]
#   Natural:  M_0 = 0,  M_{K−1} = 0
# Every row is diagonally dominant (|diag| = 2·Σ|off-diag|, or 1 vs 0) ⇒ Thomas (#59), no pivoting.

function Fit(x, y, count, boundary):
    if count < 2 or count > MaxKnots: return false
    for k in 0..count−2: if x[k+1] − x[k] ≤ ε·max(1, |x[k]|): return false
    store knots, values, count;  inverseSpan[k] = 1/h_k
    build sub, diag, sup from h and boundary.condition                     # stack bands of MaxKnots
    if !tridiagonal.Factor(sub, diag, sup, count): return false            # math::TridiagonalSolver<T, MaxKnots>
    for j in 0..Dimension−1:                                               # O(K) per channel
        r = right-hand side of channel j (rows above; start/end slope_j when Clamped, 0 when Natural)
        M[·][j] = tridiagonal.Solve(r)
    return true

function Evaluate(x):                                                     # OPTIMIZE_FOR_SPEED
    x = clamp(x, x_0, x_{K−1})
    k = (upper_bound(knots[1 .. K−1], x) index) clamped to [0, K−2]        # binary search, O(log K)
    τ = x − x_k;  h = x_{k+1} − x_k;  invH = inverseSpan[k]
    for j in 0..Dimension−1:
        d = (M_{k+1,j} − M_{k,j})·invH/6
        β = (y_{k+1,j} − y_{k,j})·invH − h·(2M_{k,j} + M_{k+1,j})/6
        value_j            = y_{k,j} + τ·(β + τ·(M_{k,j}/2 + τ·d))
        firstDerivative_j  = β + τ·(M_{k,j} + 3d·τ)
        secondDerivative_j = M_{k,j} + 6d·τ
    return { value, firstDerivative, secondDerivative }
```

Verified with python3: knots reproduced exactly, one-sided first/second derivatives at interior knots
agree to `2e-15`, clamped end slopes exact, natural end second derivatives zero, collinear knots ⇒
`M ≡ 0`, two clamped knots ⇒ exactly the cubic Hermite polynomial (`PolynomialProfile`, #55).

## Complexity & memory

- `Fit`: `O(K)` shared factorization + `O(K)` per channel ⇒ `O(Dimension·K)`; no pivoting, no iteration.
- `Evaluate`: `O(log K)` interval search + `O(Dimension)` Horner evaluations.
- Memory: `MaxKnots·(2 + 2·Dimension)` scalars in bounded `std::array`s plus the solver's three bands.
  No heap.

## Numerical / embedded notes

- Fit once, then `Evaluate` is deterministic: one binary search plus fixed arithmetic.
- Reject `h_k ≤ ε`: near-coincident knots make the rows ill-conditioned and `M` huge.
- Clamped ends fix the end slopes but leave the end second derivatives free (generally non-zero); natural
  ends zero the second derivatives but leave the slopes free — pick per application.
- A cubic spline interpolates; it does not bound the curve between knots (overshoot is possible). Check
  with `Evaluate` where limits matter.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/math/CubicSpline.hpp` — `#pragma once`, `OPTIMIZE_FOR_SPEED` on `Evaluate`, and
  `extern template class CubicSpline<float, 3, 8>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/math/CubicSpline.cpp` → `template class CubicSpline<float, 3, 8>;`
- Test: `numerical/math/test/TestCubicSpline.cpp`
- Doc: `doc/math/CubicSpline.md` (per `doc/TEMPLATE.md`) + row in `doc/math/README.md`
- CMake: `.hpp` → `target_sources(numerical.math PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.math ...)`; `TestCubicSpline.cpp` → `numerical.math_test`.
- Depends on: #59 (`TridiagonalSolver`).
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

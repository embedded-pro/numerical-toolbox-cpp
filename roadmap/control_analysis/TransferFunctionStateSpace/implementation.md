# Transfer-Function ↔ State-Space Conversion — Implementation Pseudocode

> Roadmap ref: #32 (Tier 4) · Target: `numerical/control_analysis` · Namespace `control_analysis` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
# SISO transfer function  H(s) = (b0 sⁿ + ... + bn) / (a0 sⁿ + ... + an), monic after normalisation.
template<typename T, size_t n>       # static_assert(std::is_floating_point_v<T>); instantiated for float
struct TransferFunction:
    std::array<T, n+1> numerator      # b0 .. bn  (descending powers)
    std::array<T, n+1> denominator    # a0 .. an  (a0 normalised to 1)

# Output realisation is the existing runtime model:
using Realization = math::LinearTimeInvariant<T, n, 1, 1>   # (A,B,C,D)
```

## Interface

```
# Transfer function  ->  state space:
static Realization ToControllableCanonical(const TransferFunction<T,n>& tf)
static Realization ToObservableCanonical  (const TransferFunction<T,n>& tf)

# State space  ->  transfer function (SISO):
static TransferFunction<T,n> ToTransferFunction(const Realization& sys)
```

## Algorithm (pseudocode)

```
function ToControllableCanonical(tf):             # OPTIMIZE_FOR_SPEED
    a = tf.denominator / tf.denominator[0]         # force monic
    (D, bhat) = ProperSplit(tf.numerator, a)       # if deg(num)==n: D=b0, bhat = num − b0·a
    A = 0
    for i in 0 .. n-2: A(i, i+1) = 1               # super-diagonal ones
    for j in 0 .. n-1: A(n-1, j) = -a[n-j]         # bottom row = −aₙ … −a₁
    B = UnitVectorLast(n)                          # [0 … 0 1]ᵀ
    C(0, j) = bhat[n-j] for j in 0 .. n-1          # [bₙ' … b₁']
    return Realization{A, B, C, D}

function ToObservableCanonical(tf):
    ccf = ToControllableCanonical(tf)              # OCF is the dual of CCF
    return Realization{ A: Transpose(ccf.A), B: Transpose(ccf.C),
                        C: Transpose(ccf.B), D: ccf.D }

function ToTransferFunction(A, B, C, D):
    # Faddeev–Le Verrier: characteristic poly + adjugate in one recursion
    den = CharacteristicPolynomial(A)              # a0=1, a1..an
    num = LeVerrierNumerator(A, B, C, D)           # coefficients of C·adj(sI−A)·B + D·det
    return TransferFunction{num, den}
```

## Complexity & memory

- Canonical realisation: `O(n²)` to fill the companion matrix; coefficients are direct copies.
- `ToTransferFunction` via Faddeev–Le Verrier: `O(n⁴)` — `n` steps, each a dense `O(n³)` matmul.
- Memory: `O(n²)` for `A`, `O(n)` for the coefficient arrays — all `std::array`, stack/static.

## Numerical / embedded notes

- **Proper vs strictly proper:** when `deg(num) == deg(den)` the system has direct feed-through — split
  off `D = b0/a0` by one polynomial division so the remaining part is strictly proper.
- **Normalise first:** divide by the leading denominator coefficient to make it monic; guard `a0 ≈ 0`.
- **CCF ↔ OCF duality** — the observable form is the transpose of the controllable form; implement the
  companion build once and transpose (DRY).
- **Minimality:** a canonical realisation is minimal only when numerator and denominator are coprime;
  a pole-zero cancellation yields a non-minimal (uncontrollable *or* unobservable) model — still valid,
  but flag it via item 26.
- Le Verrier is elegant but its conditioning degrades with `n`; keep to the small orders typical on an MCU.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/control_analysis/TransferFunctionStateSpace.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on the conversions, and
  `extern template class TransferFunctionStateSpace<float, 2>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/control_analysis/TransferFunctionStateSpace.cpp` →
  `template class TransferFunctionStateSpace<float, 2>;`
- Test: `numerical/control_analysis/test/TestTransferFunctionStateSpace.cpp`
- Doc: `doc/control_analysis/TransferFunctionStateSpace.md`
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestTransferFunctionStateSpace.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

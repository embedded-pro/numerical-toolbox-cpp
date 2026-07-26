# Dynamic Parameter Identification — Implementation Pseudocode

> Roadmap ref: #M22 (Tier 4) · Target: `numerical/estimators/offline` · Namespace `estimators` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof, std::size_t Params, std::size_t Samples>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class DynamicParameterIdentification:
    Matrix<T, Params, Params> A = 0         # accumulated  Σ Yᵀ Y
    Vector<T, Params>          b = 0         # accumulated  Σ Yᵀ τ
    Vector<T, Params>          parameters    # estimated base parameters â
    Rnea<T, Dof>&              model         # regressor source (injected)
```

## Interface

```
DynamicParameterIdentification(Rnea<T, Dof>& model)
void Accumulate(Vector<Dof> q, Vector<Dof> qd, Vector<Dof> qdd, Vector<Dof> tau)  # per sample
bool Estimate()                              # solve once trajectory captured
const Vector<Params>& Parameters()
void Reset()
```

## Algorithm (pseudocode)

```
function Accumulate(q, qd, qdd, tau):        # OPTIMIZE_FOR_SPEED
    # Linear-in-parameters regressor from inverse dynamics:  Y(q,q̇,q̈) · a = τ
    Y = model.Regressor(q, qd, qdd)          # (Dof x Params)
    A += Yᵀ * Y                               # fold sample into normal equations
    b += Yᵀ * tau                             # never store the full stacked system

function Estimate():                          # OPTIMIZE_FOR_SPEED
    # Least-squares base-parameter solve:  (Σ YᵀY) â = Σ Yᵀτ
    if not PositiveDefinite(A): return false  # trajectory not exciting enough
    parameters = solvers::SolveSystem(A, b)   # or QR on the stacked form
    return true

function Reset():
    A = 0 ;  b = 0
```

## Complexity & memory

- Accumulate: `O(Dof·Params²)` per sample; runs online over the excitation trajectory.
- Estimate: one `O(Params³)` solve after the trajectory is captured.
- Memory: `O(Params²)` for the running normal equations — **independent of sample count**; no heap.

## Numerical / embedded notes

- Only the **base parameters** (identifiable combinations) are observable; the full inertial set is
  rank-deficient. A non-positive-definite `A` means the trajectory under-excites some parameters —
  enrich the excitation (more frequencies) rather than regularizing blindly.
- Accumulating `Σ YᵀY` keeps memory constant but **squares the condition number**; prefer a **QR**
  update on the stacked `[Y; …]` when conditioning is marginal.
- Filter `q̈` (and torque) consistently — differentiation noise biases the regressor.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/estimators/offline/DynamicParameterIdentification.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Accumulate`/`Estimate`, and
  `extern template class DynamicParameterIdentification<float, 2, Params, 20>;` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/estimators/offline/DynamicParameterIdentification.cpp` →
  `template class DynamicParameterIdentification<float, 2, Params, 20>;`
- Test: `numerical/estimators/offline/test/TestDynamicParameterIdentification.cpp`
- Doc: `doc/estimators/DynamicParameterIdentification.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestDynamicParameterIdentification.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

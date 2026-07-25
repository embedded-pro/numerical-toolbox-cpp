# Feedforward / 2-DOF Controller — Implementation Pseudocode

> Roadmap ref: #7 (Tier 2) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>
interface Feedforward:                    # injected map r -> u_ff
    T Evaluate(T reference) const

template<typename T>
interface FeedbackLaw:                     # injected e -> u_fb
    T Process(T error)

template<typename T>                 # static_assert(std::is_floating_point_v<T>); instantiated for float
class Feedforward2Dof:
    Feedforward<T>&  feedforward           # constructor-injected
    FeedbackLaw<T>&  feedback              # constructor-injected
    Saturation<T>    outputClamp           # actuator bound on the summed command
```

## Interface

```
Feedforward2Dof(Feedforward<T>& ff, FeedbackLaw<T>& fb, Saturation<T> clamp)
T    Compute(T reference, T measurement)   # hot path
void Reset()
```

## Algorithm (pseudocode)

```
function Compute(r, y):                     # OPTIMIZE_FOR_SPEED
    # Two degrees of freedom: shape response (ff) and reject error (fb)
    uFeedforward = feedforward.Evaluate(r)          # model-based, open-loop term
    error        = r - y
    uFeedback    = feedback.Process(error)          # closed-loop correction
    return outputClamp.Clamp(uFeedforward + uFeedback)
```

## Complexity & memory

- Time: `O(1)` in the wrapper, plus the cost of the injected feedforward and feedback blocks.
- Memory: `O(1)` in the wrapper; all state lives inside the injected components.

## Numerical / embedded notes

- **Dependency inversion:** the wrapper owns neither the feedforward map nor the feedback law —
  both arrive by reference (constructor injection), so any `Feedforward`/`FeedbackLaw`
  implementation composes without edits (Open/Closed).
- The feedforward term is open-loop: it does not affect loop stability margins, so it can be tuned
  for tracking independently of the feedback tuning.
- A static/gain feedforward (`u_ff = Kff·r`) is the trivial `Feedforward` implementation; an
  inverse-plant model is the high-performance one.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/Feedforward2Dof.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class Feedforward2Dof<float>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/Feedforward2Dof.cpp` →
  `template class Feedforward2Dof<float>;`
- Test: `numerical/controllers/implementations/test/TestFeedforward2Dof.cpp`
- Doc: `doc/controllers/Feedforward2Dof.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestFeedforward2Dof.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

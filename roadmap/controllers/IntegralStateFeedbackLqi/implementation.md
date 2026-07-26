# Integral State Feedback (LQI / Servo) — Implementation Pseudocode

> Roadmap ref: #20 (Tier 3) · Target: `numerical/controllers` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class IntegralStateFeedbackLqi:
    # Gains split across plant states and integral states:
    math::Matrix<T, InputSize, StateSize>  Kx        # state-feedback gain
    math::Matrix<T, InputSize, OutputSize> Ki        # integral gain
    math::Vector<T, OutputSize>            integral  # accumulated tracking error
    T                                      sampleTime
```

## Interface

```
# Gains from an LQR solve on the augmented plant (design-time):
IntegralStateFeedbackLqi(const Matrix& Kx, const Matrix& Ki, T Ts)

# Or built from plant + weights, augmenting internally then delegating to Lqr:
IntegralStateFeedbackLqi(const LinearTimeInvariant& plant,
                         const StateWeight& Q, const InputWeight& R, T Ts)

InputVector ComputeControl(const StateVector& x,
                           const OutputVector& reference,
                           const OutputVector& measured)      # hot path
void Reset()
```

## Algorithm (pseudocode)

```
# --- Design (once) -------------------------------------------------
augment(plant):
    # Add integral-of-error states x_i[k+1] = x_i[k] + (r - y)·Ts
    #   [ x  ]      [ A      0 ] [ x  ]   [ B ]        [ 0  ]
    #   [ x_i]  ->  [ -C·Ts  I ] [ x_i] + [ 0 ] u  +   [ Ts ] r
    Aa = [[A, 0], [-C*Ts, I]]
    Ba = [[B], [0]]
    return (Aa, Ba)

design(plant, Q, R):
    (Aa, Ba) = augment(plant)
    Ka       = Lqr(Aa, Ba, Q, R).GetGain()       # DARE solve on the augmented plant
    (Kx, Ki) = split(Ka)                          # first StateSize cols / last OutputSize cols

# --- Run (hot) -----------------------------------------------------
function ComputeControl(x, r, y):                 # OPTIMIZE_FOR_SPEED
    error    = r - y
    integral = integral + error * sampleTime      # accumulate tracking error
    # u = -Kx·x - Ki·∫e
    return (Kx * x) * (-1) + (Ki * integral) * (-1)
```

## Complexity & memory

- Design: one DARE solve on the augmented `(StateSize+OutputSize)` system — `O((n+p)³)`.
- `ComputeControl`: `O(InputSize·(StateSize+OutputSize))` — two matrix-vector products.
- Memory: `O(...)` for the two gains plus `O(OutputSize)` for the integral state — all static.

## Numerical / embedded notes

- The integral states are exactly what removes **steady-state error**: at equilibrium the
  integrator stops changing only when `r − y = 0`, guaranteeing zero offset to constant references.
- Add **anti-windup**: clamp `integral` (or use `SaturationRateLimiter` on the output and
  back-calculate) so a saturated actuator does not let the integral run away.
- Reuse `Lqr` and `DiscreteAlgebraicRiccatiEquation` for the gain solve — do not duplicate the
  Riccati iteration (DRY).
- Weight the integral states in `Q` to trade steady-state convergence speed against overshoot.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/IntegralStateFeedbackLqi.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeControl`, and
  `extern template class IntegralStateFeedbackLqi<float, StateSize, InputSize, OutputSize>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/IntegralStateFeedbackLqi.cpp` →
  `template class IntegralStateFeedbackLqi<float, StateSize, InputSize, OutputSize>;`
- Test: `numerical/controllers/implementations/test/TestIntegralStateFeedbackLqi.cpp`
- Doc: `doc/controllers/IntegralStateFeedbackLqi.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestIntegralStateFeedbackLqi.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

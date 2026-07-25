# Alpha-Beta / Alpha-Beta-Gamma Filter — Implementation Pseudocode

> Roadmap ref: #8 (Tier 2) · Target: `numerical/filters/active` · Namespace `filters` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Order>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class AlphaBetaFilter:                  # Order = 2 (α-β) or 3 (α-β-γ)
    T                 Ts               # sample period
    T                 alpha, beta      # position / velocity gains
    T                 gamma            # acceleration gain (Order == 3 only)
    Vector<T, Order>  state            # [position, velocity, (acceleration)]
    bool              initialized = false
```

## Interface

```
AlphaBetaFilter(T alpha, T beta, T Ts)                 # Order == 2
AlphaBetaFilter(T alpha, T beta, T gamma, T Ts)        # Order == 3
T      Filter(T measuredPosition)      # hot path: predict + correct, returns position
Vector<T, Order> State() const         # full [pos, vel, (acc)] estimate
void   Reset(T position = 0)
static Gains GainsFromTrackingIndex(T lambda)   # Kalata steady-state design
```

## Algorithm (pseudocode)

```
function Filter(z):                          # OPTIMIZE_FOR_SPEED
    if not initialized:                      # seed position, zero rates
        state[0] = z; initialized = true; return z

    # --- Predict (constant-velocity / constant-accel kinematics) ---
    p = state[0] + Ts*state[1] + (Order==3 ? 0.5*Ts*Ts*state[2] : 0)
    v = state[1] + (Order==3 ? Ts*state[2] : 0)
    a = state[2]                             # unchanged (Order == 3)

    # --- Correct with position residual ---
    r = z - p                                # innovation
    state[0] = p + alpha*r
    state[1] = v + (beta / Ts)*r
    if Order == 3:
        state[2] = a + (2*gamma / (Ts*Ts))*r
    return state[0]

function GainsFromTrackingIndex(lambda):     # α-β, Kalata 1984
    r     = (4 + lambda - sqrt(8*lambda + lambda*lambda)) / 4   # smoothing root
    alpha = 1 - r*r
    beta  = 2*(2 - alpha) - 4*sqrt(1 - alpha)
    return {alpha, beta}
```

## Complexity & memory

- Time: `O(1)` per sample — a handful of MACs, no matrix inverse, no covariance update.
- Memory: `O(Order)` — 2 or 3 state words plus the fixed gains.

## Numerical / embedded notes

- Deterministic drop-in for a steady-state Kalman filter: gains are precomputed, so there is
  **no online covariance propagation** — ideal for tight ISR budgets.
- Stability (α-β) requires `0 < alpha < 1` and `0 < beta < 4 - 2*alpha` (Simpson's triangle).
- The **tracking index** `lambda = sqrt(process_var)*Ts² / sqrt(meas_var)` sets the smoothing/lag
  trade-off; `GainsFromTrackingIndex` gives the critically-damped α-β gains from Kalata.
- Precompute `beta/Ts` and `2*gamma/Ts²` as constants so the hot path stays MAC-only.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/active/AlphaBetaFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class AlphaBetaFilter<float, Order>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/active/AlphaBetaFilter.cpp` →
  `template class AlphaBetaFilter<float, Order>;`
- Test: `numerical/filters/active/test/TestAlphaBetaFilter.cpp`
- Doc: `doc/filters/active/AlphaBetaFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestAlphaBetaFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

# Active Disturbance Rejection Control (ADRC + ESO) — Implementation Pseudocode

> Roadmap ref: #36 (Tier 4) · Target: `numerical/robust_control` · Namespace `robust_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Order>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class ActiveDisturbanceRejectionControl:
    # Extended State Observer: Order plant states + 1 total-disturbance state
    math::Vector<T, Order + 1>  xhat          # [ ŷ, ẏ̂, ..., f̂ ]  (f̂ = total disturbance)
    math::Vector<T, Order + 1>  observerGain  # β, bandwidth-parameterized
    math::Vector<T, Order>      controlGain   # k (kp, kd, ...), bandwidth-parameterized
    T                           b0            # input-gain estimate
    T                           sampleTime    # Ts
    T                           appliedPrev   # last u (drives highest derivative)
```

## Interface

```
ActiveDisturbanceRejectionControl(T observerBandwidth,       # ω_o
                                  T controlBandwidth,         # ω_c
                                  T b0, T Ts)

T Compute(T reference, T measuredOutput)                      # hot path
void  Reset()
static Vector<T,Order+1> ObserverGainFromBandwidth(T wo)      # β_i = C(n+1,i)·ω_o^i
static Vector<T,Order>   ControlGainFromBandwidth(T wc)       # place at -ω_c
```

## Algorithm (pseudocode)

```
function Compute(r, y):                          # OPTIMIZE_FOR_SPEED
    # --- ESO: correct on the output error, then predict (discrete Euler) ---
    e = y - xhat[0]                              # output estimation error
    for i in 0 .. Order:                         # inject correction β_i·e into every state
        xhat[i] += Ts * observerGain[i] * e
    for i in 0 .. Order - 1:                      # chain of integrators
        xhat[i] += Ts * xhat[i + 1]
    xhat[Order - 1] += Ts * b0 * appliedPrev      # b0·u drives the highest derivative
    # (the disturbance state xhat[Order] moves only via its correction term)

    # --- control law: reject total disturbance f̂ = xhat[Order] ---
    u0 = controlGain[0] * (r - xhat[0])          # kp·(r − ŷ)
    for i in 1 .. Order - 1:
        u0 -= controlGain[i] * xhat[i]           # − kd·(derivative estimates)
    u = (u0 - xhat[Order]) / b0                  # subtract estimated disturbance, scale by 1/b0
    appliedPrev = u
    return u
```

## Complexity & memory

- `Compute`: `O(Order)` — the ESO and control law are linear sweeps over `Order + 1` states.
- Design-time: `O(Order)` binomial/pole formulas for the gains.
- Memory: `O(Order)` for the estimate and gain vectors — all static, no heap.

## Numerical / embedded notes

- **Bandwidth parameterization (Gao):** collapse all tuning to two knobs — observer bandwidth `ω_o`
  and control bandwidth `ω_c` — by placing all ESO poles at `−ω_o` and all control poles at `−ω_c`.
- **Near model-free:** only `b0` and the two bandwidths are needed; `f̂` absorbs unmodeled dynamics
  and external disturbance, so a rough `b0` still works.
- Typically `ω_o ≈ 3–10·ω_c`: too high amplifies measurement noise and causes ESO peaking, too low
  leaves the disturbance unrejected.
- The Euler discretization shown is simplest; a ZOH/discrete ESO improves accuracy at low sample rates.
- Reuse item 19 (observer) structure and `math::Vector`; guard against **ESO peaking** at startup
  (large initial `e`) by limiting the initial estimate or scheduling `ω_o`.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/robust_control/ActiveDisturbanceRejection.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`, and
  `extern template class ActiveDisturbanceRejectionControl<float, Order>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/robust_control/ActiveDisturbanceRejection.cpp` →
  `template class ActiveDisturbanceRejectionControl<float, Order>;`
- Test: `numerical/robust_control/test/TestActiveDisturbanceRejection.cpp`
- Doc: `doc/robust_control/ActiveDisturbanceRejection.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestActiveDisturbanceRejection.cpp` → the `_test` target.
- New module: create `numerical/robust_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/robust_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

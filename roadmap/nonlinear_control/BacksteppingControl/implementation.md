# Backstepping Controller — Implementation Pseudocode

> Roadmap ref: #41 (Tier 4) · Target: `numerical/nonlinear_control` · Namespace `nonlinear_control` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
# Injected strict-feedback model exposes per-stage drift/gain (DIP):
template<typename T, std::size_t Order>   # pure interface
interface StrictFeedbackModel:
    T Drift(int i, const StateVector& x) const          # f_i(x_1..x_{i+1})
    T Gain (int i, const StateVector& x) const          # g_i(x_1..x_{i+1}), ≠ 0
    T VirtualDerivative(int i, const StateVector& x, T alpha) const  # α̇_{i-1}

template<typename T, std::size_t Order>   # static_assert(std::is_floating_point_v<T>); instantiated for float
class BacksteppingControl:
    const StrictFeedbackModel<T, Order>& model
    std::array<T, Order> gains          # k_1..k_n > 0
```

## Interface

```
BacksteppingControl(const StrictFeedbackModel<T,Order>& model,
                    const std::array<T,Order>& gains)

T ComputeControl(const StateVector& x, const Reference& ref)       # hot path
void  Reset()
```

## Algorithm (pseudocode)

```
function ComputeControl(x, ref):                       # OPTIMIZE_FOR_SPEED
    # Strict-feedback chain:  ẋ_i = f_i + g_i · x_{i+1},  ẋ_n = f_n + g_n · u
    z[0]        = x[0] - ref.value                      # tracking error, stage 1
    alphaDot    = ref.derivative                        # α̇_0 = ṙ (feedforward)
    for i in 0 .. Order-1:
        f = model.Drift(i, x)
        g = model.Gain (i, x)                           # controllability: g ≠ 0
        cross = (i == 0) ? 0 : model.Gain(i-1, x) * z[i-1]   # Lyapunov cross term

        # virtual control that makes V̇_i = -Σ k·z²  along the chain
        alpha_i = ( alphaDot - f - gains[i]*z[i] - cross ) / g

        if i < Order-1:
            z[i+1]   = x[i+1] - alpha_i                 # next-stage error
            alphaDot = model.VirtualDerivative(i, x, alpha_i)   # ∂α_i/∂x·ẋ + ∂α_i/∂t
        else:
            return alpha_i                              # α_n is the actual input u
```

## Complexity & memory

- `ComputeControl`: `O(Order)` — a single forward sweep through the integrator chain.
- Memory: `O(Order)` for the error stack `z` and the gains; static, no heap, no recursion.

## Numerical / embedded notes

- **Controllability guard:** each `g_i(x)` divides the virtual control; assert `|g_i| ≥ ε` and keep
  the state in the region where the chain is invertible.
- The **hard part** is `α̇_{i-1}`: the virtual control's time derivative must be propagated
  analytically (chain rule) or the design loses its stability guarantee — do *not* differentiate it
  numerically on-target (noise amplification). Inject it via `VirtualDerivative`.
- Lyapunov certificate `V = ½·Σ z_i²` decreases monotonically ⇒ provable stability; the gains
  `k_i > 0` set each stage's decay rate independently.
- Combine with online estimation for unknown parameters ⇒ *adaptive backstepping* (bridges to MRAC).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/nonlinear_control/BacksteppingControl.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeControl`, and
  `extern template class BacksteppingControl<float, Order>;`
  under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/nonlinear_control/BacksteppingControl.cpp` →
  `template class BacksteppingControl<float, Order>;`
- Test: `numerical/nonlinear_control/test/TestBacksteppingControl.cpp`
- Doc: `doc/nonlinear_control/BacksteppingControl.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestBacksteppingControl.cpp` → the `_test` target.
- New module: create `numerical/nonlinear_control/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add `test/`, register in `numerical/CMakeLists.txt`, add `doc/nonlinear_control/`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

# Friction Compensation (Coulomb + Viscous + Stribeck) — Implementation Pseudocode

> Roadmap ref: #57 (Tier 1) · Target: `numerical/controllers/implementations` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```cpp
template<typename T>                        # static_assert(std::is_floating_point_v<T>); instantiated for float
struct FrictionParameters:                  # one per axis (joint, motor, linear stage)
    T coulomb          # F_c  — kinetic friction magnitude
    T stiction         # F_s  — static/breakaway level (>= F_c)
    T viscous          # F_v  — velocity-proportional coefficient
    T stribeckVelocity # v_s  — width of the Stribeck dip (> 0)
    T stribeckShape    # delta — exponent, typically 1 or 2
    T smoothingVelocity# eps  — zero-crossing smoothing width (> 0)
    T maxCompensation  # |τ_f| cap — over-compensation can drive the axis (> 0)

template<typename T, std::size_t Axes> # static_assert(std::is_floating_point_v<T>); instantiated for float
class FrictionCompensation:
    std::array<FrictionParameters<T>, Axes> params
```

## Interface

```cpp
FrictionCompensation(const std::array<FrictionParameters<T>, Axes>& params)

# Feedforward friction torque (force, for a linear axis) to add to the control law:
Vector<T, Axes> Compute(const Vector<T, Axes>& velocity) const           # hot path

# Single-axis helper (also reused by the vector form):
static T AxisTorque(const FrictionParameters<T>& p, T velocity)
```

## Algorithm (pseudocode)

```text
function AxisTorque(p, v):                         # OPTIMIZE_FOR_SPEED
    # Stribeck curve: stiction blends down to Coulomb as |v| grows
    fall  = exp( -(|v| / p.stribeckVelocity) ^ p.stribeckShape )
    level = p.coulomb + (p.stiction - p.coulomb) * fall
    # smooth sign (tanh boundary layer) avoids chattering at v = 0
    dir   = tanh(v / p.smoothingVelocity)
    return clamp(level * dir + p.viscous * v, −p.maxCompensation, p.maxCompensation)

function Compute(velocity):                              # OPTIMIZE_FOR_SPEED
    for i in 0 .. Axes-1:
        tau[i] = AxisTorque(params[i], velocity[i])
    return tau
```

## Complexity & memory

- `Compute`: `O(Axes)` — one `exp` and one `tanh` per axis, no allocation.
- Memory: `O(Axes)` parameter structs, all static/stack-resident.

## Numerical / embedded notes

- **Feedforward, not feedback:** add the result to the controller output
  (`τ_cmd = τ_control + Compute(v)`); it composes with any torque/current loop — PID, a FOC current
  reference, or robotics-toolbox-cpp's PD + gravity and computed-torque controllers.
- Feed **desired** velocity for a noise-free command, or **measured** velocity for accuracy — the
  smoothing width `eps` trades tracking sharpness against chattering either way.
- The ideal `sign(v)` is replaced by `tanh(v/eps)`: too small an `eps` reintroduces limit cycles
  near zero velocity; too large under-compensates stiction on slow moves.
- Over-compensation is worse than under (it can drive the axis), so the output is clamped to
  `maxCompensation`; validate `stiction >= coulomb`, `stribeckVelocity > 0`, `smoothingVelocity > 0`
  and `maxCompensation > 0` at construction.
- The same model can be folded into a plant model (robotics-toolbox-cpp adds it to its chain inverse
  dynamics), and its parameters are linear in `(F_c, F_v)` for fixed `v_s`, so they can be identified by
  least squares together with other linear parameters (`IncrementalQrLeastSquares`, #63).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/implementations/FrictionCompensation.hpp` — `#pragma once`,
  `OPTIMIZE_FOR_SPEED` on `Compute`/`AxisTorque`, and
  `extern template class FrictionCompensation<float, 1>;` / `<float, 3>` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/implementations/FrictionCompensation.cpp` →
  `template class FrictionCompensation<float, 1>;` and `<float, 3>`.
- Test: `numerical/controllers/implementations/test/TestFrictionCompensation.cpp`
- Doc: `doc/controllers/FrictionCompensation.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources(numerical.controllers.implementations ...)`;
  `TestFrictionCompensation.cpp` → `numerical.controllers_test`.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".

# Friction Compensation (Coulomb + Viscous + Stribeck) — Implementation Pseudocode

> Roadmap ref: #M4 (Tier 1) · Target: `numerical/dynamics` · Namespace `dynamics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T>                        # static_assert(std::is_floating_point_v<T>); instantiated for float
struct JointFrictionParameters:             # one per joint
    T coulomb          # F_c  — kinetic friction magnitude
    T stiction         # F_s  — static/breakaway level (>= F_c)
    T viscous          # F_v  — velocity-proportional coefficient
    T stribeckVelocity # v_s  — width of the Stribeck dip (> 0)
    T stribeckShape    # delta — exponent, typically 1 or 2
    T smoothingVelocity# eps  — zero-crossing smoothing width (> 0)

template<typename T, std::size_t NumJoints> # static_assert(std::is_floating_point_v<T>); instantiated for float
class FrictionCompensation:
    std::array<JointFrictionParameters<T>, NumJoints> params
```

## Interface

```
FrictionCompensation(const std::array<JointFrictionParameters<T>, NumJoints>& params)

# Feedforward friction torque to add to the control law:
Vector<T, NumJoints> Compute(const Vector<T, NumJoints>& qDot) const           # hot path

# Single-joint helper (also reused by the vector form):
static T JointTorque(const JointFrictionParameters<T>& p, T qDot)
```

## Algorithm (pseudocode)

```
function JointTorque(p, v):                         # OPTIMIZE_FOR_SPEED
    # Stribeck curve: stiction blends down to Coulomb as |v| grows
    fall  = exp( -(|v| / p.stribeckVelocity) ^ p.stribeckShape )
    level = p.coulomb + (p.stiction - p.coulomb) * fall
    # smooth sign (tanh boundary layer) avoids chattering at v = 0
    dir   = tanh(v / p.smoothingVelocity)
    return level * dir + p.viscous * v

function Compute(qDot):                              # OPTIMIZE_FOR_SPEED
    for i in 0 .. NumJoints-1:
        tau[i] = JointTorque(params[i], qDot[i])
    return tau
```

## Complexity & memory

- `Compute`: `O(NumJoints)` — one `exp` and one `tanh` per joint, no allocation.
- Memory: `O(NumJoints)` parameter structs, all static/stack-resident.

## Numerical / embedded notes

- **Feedforward, not feedback:** add the result to the controller output
  (`τ_cmd = τ_control + Compute(q̇)`); it composes with PD+gravity (M5) and computed-torque (M12).
- Feed **desired** velocity for a noise-free command, or **measured** velocity for accuracy — the
  smoothing width `eps` trades tracking sharpness against chattering either way.
- The ideal `sign(v)` is replaced by `tanh(v/eps)`: too small an `eps` reintroduces limit cycles
  near zero velocity; too large under-compensates stiction on slow moves.
- Over-compensation is worse than under (it can drive the joint), so cap each term and validate
  `stiction >= coulomb`, `stribeckVelocity > 0`, `smoothingVelocity > 0` at construction.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/dynamics/FrictionCompensation.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Compute`/`JointTorque`, and
  `extern template class FrictionCompensation<float, NumJoints>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/dynamics/FrictionCompensation.cpp` →
  `template class FrictionCompensation<float, NumJoints>;`
- Test: `numerical/dynamics/test/TestFrictionCompensation.cpp`
- Doc: `doc/dynamics/FrictionCompensation.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestFrictionCompensation.cpp` → the `_test` target.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".

# PD + Gravity Compensation — Implementation Pseudocode

> Roadmap ref: #M5 (Tier 1) · Target: `numerical/controllers/manipulator` · Namespace `controllers` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t Dof>    # static_assert(std::is_floating_point_v<T>); instantiated for float
class PdGravityCompensation:
    const dynamics::EulerLagrangeDynamics<T, Dof>& model   # only g(q) is queried
    math::SquareMatrix<T, Dof> Kp        # proportional gain (SPD)
    math::SquareMatrix<T, Dof> Kd        # derivative gain (SPD)
```

## Interface

```
# Dynamics model injected (DIP); the set-point regulator needs only its gravity term:
PdGravityCompensation(const EulerLagrangeDynamics<T,Dof>& model,
                      const SquareMatrix& Kp, const SquareMatrix& Kd)

Vector<T,Dof> ComputeTorque(const StateVector& q,
                            const StateVector& qDot,
                            const StateVector& qDesired)     # hot path
```

## Algorithm (pseudocode)

```
function ComputeTorque(q, qDot, qd):              # OPTIMIZE_FOR_SPEED
    # set-point regulation: qd is constant, so the desired velocity is zero
    e = qd - q
    # τ = Kp·e − Kd·q̇ + g(q)
    g = model.ComputeGravityTerms(q)
    return Kp * e - Kd * qDot + g
```

## Complexity & memory

- Time: `O(Dof²)` for the two gain products (`O(Dof)` when the gains are diagonal); gravity
  evaluation is `O(Dof)`–`O(Dof²)` via the injected RNEA / Euler-Lagrange model.
- Memory: `O(Dof²)` for the two gains; no dynamic state, no heap.

## Numerical / embedded notes

- Globally asymptotically stable for *any* SPD `Kp`, `Kd` (Takegaki–Arimoto): the Lyapunov function
  `V = ½q̇ᵀM(q)q̇ + ½eᵀKp e` decreases as the `Kd·q̇` term drains kinetic energy.
- The `g(q)` term removes the static "droop"; without it a gravity-loaded joint settles with a
  steady offset `e_ss = Kp⁻¹·g`.
- **Set-point regulator only** — `qd` is constant. For trajectory tracking use `ComputedTorqueControl`.
- Diagonal `Kp`, `Kd` decouple the joints and keep the hot path to `Dof` multiplies.
- Only `g(q)` is drawn from the injected model — the same interface used by the full computed-torque
  law, so a single dynamics object serves both controllers (ISP / DIP).
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/controllers/manipulator/PdGravityCompensation.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `ComputeTorque`, and
  `extern template class PdGravityCompensation<float, Dof>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/controllers/manipulator/PdGravityCompensation.cpp` →
  `template class PdGravityCompensation<float, Dof>;`
- Test: `numerical/controllers/manipulator/test/TestPdGravityCompensation.cpp`
- Doc: `doc/controllers/manipulator/PdGravityCompensation.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestPdGravityCompensation.cpp` → the `_test` target.
- New module: create `numerical/controllers/manipulator/CMakeLists.txt` via `numerical_add_header_library(...)`,
  add a `test/` subdir, register it in `numerical/controllers/CMakeLists.txt`, and add a
  `doc/controllers/manipulator/` folder.
- Generic pattern: see `roadmap/README.md` → "Deployment shape".

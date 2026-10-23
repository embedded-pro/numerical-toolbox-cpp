# Symplectic Integrators (Semi-Implicit Euler, Velocity Verlet) — Implementation Pseudocode

> Roadmap ref: #58 (Tier 1) · Target: `numerical/solvers` · Namespace `solvers` · Type: `float` (templated on `T`, instantiated for `float` only)

Fixed-step integrators for second-order systems `q̈ = a(q, q̇, u)` — mechanical plants (robot arms,
pendulums, drones, motor + load) simulated in model-in-the-loop tests or run as on-target observers.
Unlike the existing `RungeKutta4`/`DormandPrince45`, which integrate a first-order `OdeSystem`, these
exploit the position/velocity split: semi-implicit Euler and velocity Verlet keep the energy error of a
conservative system bounded instead of drifting.

## Data structures

```cpp
template<typename T, std::size_t Dimension, std::size_t InputSize>   # static_assert(std::is_floating_point_v<T>)
class SecondOrderSystem:                       # pure interface
    virtual ~SecondOrderSystem() = default
    virtual math::Vector<T, Dimension> Acceleration(const math::Vector<T, Dimension>& q,
                                                    const math::Vector<T, Dimension>& qDot,
                                                    const math::Vector<T, InputSize>& u) = 0

template<typename T, std::size_t Dimension>
struct SecondOrderState:
    math::Vector<T, Dimension> q, qDot

template<typename T, std::size_t Dimension, std::size_t InputSize>   # instantiated for float
class SemiImplicitEuler:                       # also called symplectic Euler
    SecondOrderSystem<T, Dimension, InputSize>& system
    T h                                        # step size > 0

template<typename T, std::size_t Dimension, std::size_t InputSize>
class VelocityVerlet:                          # Störmer–Verlet, kick-drift-kick form
    SecondOrderSystem<T, Dimension, InputSize>& system
    T h
```

## Interface

```cpp
SemiImplicitEuler(SecondOrderSystem<T, Dimension, InputSize>& system, T h)
SecondOrderState<T, Dimension> Step(const SecondOrderState<T, Dimension>& x, const InputVector& u)   # hot path

VelocityVerlet(SecondOrderSystem<T, Dimension, InputSize>& system, T h)
SecondOrderState<T, Dimension> Step(const SecondOrderState<T, Dimension>& x, const InputVector& u)   # hot path
```

`u` is held constant over the step (zero-order hold), matching `RungeKutta4::Step`.

## Algorithm (pseudocode)

```text
function SemiImplicitEuler::Step(x, u):        # OPTIMIZE_FOR_SPEED — one evaluation
    qDot = x.qDot + h·system.Acceleration(x.q, x.qDot, u)
    q    = x.q + h·qDot                        # uses the NEW velocity — the symplectic part
    return { q, qDot }

function VelocityVerlet::Step(x, u):           # OPTIMIZE_FOR_SPEED — two evaluations
    vHalf = x.qDot + (h/2)·system.Acceleration(x.q, x.qDot, u)        # kick
    q     = x.q + h·vHalf                                             # drift
    qDot  = vHalf + (h/2)·system.Acceleration(q, vHalf, u)            # kick
    return { q, qDot }
```

## Complexity & memory

- Semi-implicit Euler: one `Acceleration` call and `O(Dimension)` arithmetic per step.
- Velocity Verlet: two `Acceleration` calls per step (the second could be reused as the next step's
  first only while `u` is unchanged — not done, to keep `Step` pure).
- Memory: two state copies; no heap.

## Numerical / embedded notes

- **Order and energy:** semi-implicit Euler is first order, velocity Verlet second order; for a
  conservative, position-only acceleration both are symplectic, so the energy error stays bounded for
  all time (no secular drift), unlike explicit Euler (grows) or RK4 (slow drift).
- With velocity-dependent forces (damping, Coriolis) neither method is exactly symplectic; Verlet
  evaluates them at the half-step velocity, which keeps second order. Dissipation is then physical, not
  numerical.
- Stability: for an oscillator of angular frequency `ω`, both are stable for `h·ω < 2`.
- The interface call is virtual; acceptable off the hard real-time path (simulation, MIL tests). Bind
  the concrete system type where the step runs in an ISR.
- Float-only: `static_assert(std::is_floating_point_v<T>)`.

## Deployment

- Header: `numerical/solvers/SymplecticIntegrators.hpp` (+ `numerical/solvers/SecondOrderSystem.hpp`, pure
  interface) — `#pragma once`, `OPTIMIZE_FOR_SPEED` on both `Step`s, and
  `extern template class SemiImplicitEuler<float, 1, 1>;` / `VelocityVerlet<float, 1, 1>` under
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/solvers/SymplecticIntegrators.cpp` → the same instantiations.
- Test: `numerical/solvers/test/TestSymplecticIntegrators.cpp`
- Doc: `doc/solvers/SymplecticIntegrators.md` (per `doc/TEMPLATE.md`) + row in `doc/solvers/README.md`
- CMake: `.hpp` → `target_sources(numerical.solver PRIVATE ...)`; `.cpp` →
  `numerical_add_coverage_sources(numerical.solver ...)`; `TestSymplecticIntegrators.cpp` →
  `numerical.solvers_test`.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

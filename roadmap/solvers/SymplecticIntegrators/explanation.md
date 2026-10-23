# Symplectic Integrators — Overview

## What it is
Two fixed-step methods for mechanical systems written as "acceleration from position, velocity and
input": semi-implicit (symplectic) Euler and velocity Verlet. They advance position and velocity in an
interleaved way that respects the geometry of mechanics.

## Why it matters (embedded)
Simulated plants are how controllers get tested before hardware: robot arms, pendulums, drones, motor
plus load. With a general-purpose integrator the simulated energy slowly creeps up or down, so a
lightly damped plant appears more (or less) stable than it is after a long run. Symplectic methods keep
the energy error bounded forever at the cost of one or two acceleration evaluations per step — cheaper
than RK4's four.

## How it works (intuition)
Semi-implicit Euler first updates the velocity from the current acceleration, then moves the position
with that *new* velocity; the tiny change from explicit Euler turns a spiralling phase portrait into a
closed one. Velocity Verlet gives the velocity half a kick, drifts the position a full step, then gives
the second half-kick with the acceleration at the new position — symmetric in time, hence second order.

## Key parameters
- **Step size `h`** — accuracy and stability (stable for `h·ω < 2` on an oscillator of frequency `ω`).
- **Method** — Euler (one evaluation, first order) or Verlet (two evaluations, second order).

## Reference
E. Hairer, C. Lubich, G. Wanner, *Geometric Numerical Integration*, 2nd ed. (2006), Ch. I and VI;
L. Verlet, *Physical Review*, 159(1), 1967.

## See also
`RungeKuttaIntegrators` and `DormandPrince45` (first-order systems, higher accuracy per step);
robotics-toolbox-cpp's forward-dynamics integrator, which binds these to its articulated-body dynamics.

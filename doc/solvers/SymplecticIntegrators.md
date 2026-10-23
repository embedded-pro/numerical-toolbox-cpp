# Symplectic Integrators (Semi-Implicit Euler + Velocity Verlet)

## Overview & Motivation

Mechanical plants (robot arms, pendulums, drones, a motor driving a load) are naturally second-order systems: $\ddot{q} = a(q, \dot{q}, u)$, with acceleration computed from position, velocity and input. They are simulated before hardware exists, so controllers can be tested in model-in-the-loop runs or run on-target as observers.

A general-purpose integrator treats this as a first-order system and ignores its structure. Over a long run the simulated energy creeps up or down, so a lightly damped plant looks more (or less) stable than it really is. **Symplectic** integrators advance position and velocity in an interleaved order that respects the geometry of mechanics. For a conservative system the energy error stays bounded forever, at one or two acceleration evaluations per step — fewer than the four of RK4.

Two fixed-step methods are provided: **semi-implicit (symplectic) Euler**, first order and one evaluation, and **velocity Verlet** in kick–drift–kick form, second order and two evaluations.

## Mathematical Theory

### Problem Statement

State $(q, v)$ with $v = \dot{q}$. The input $u$ is held constant over the step (zero-order hold), and the step size $h > 0$ is fixed.

### Semi-Implicit Euler

$$
v^+ = v + h\,a(q, v, u), \qquad q^+ = q + h\,v^+
$$

The only change from explicit Euler is that the position update uses the *new* velocity. Local error is $O(h^2)$, global error $O(h)$.

### Velocity Verlet (Kick–Drift–Kick)

$$
v_{1/2} = v + \tfrac{h}{2}\,a(q, v, u), \qquad q^+ = q + h\,v_{1/2}, \qquad v^+ = v_{1/2} + \tfrac{h}{2}\,a(q^+, v_{1/2}, u)
$$

The step is symmetric in time, which cancels the odd error terms: global error $O(h^2)$. For constant acceleration it is exact, since $v^+ = v + h\,a$ and $q^+ = q + h\,v + \tfrac{h^2}{2}a$. Semi-implicit Euler is not: it gives $q^+ = q + h\,v + h^2 a$.

### Why Energy Stays Bounded

For a position-only force, $a = -\nabla U(q)$, each method is a composition of shears (a velocity update depending on $q$, a position update depending on $v$), each with unit Jacobian determinant. The step map therefore preserves phase-space area and is *symplectic*. Backward error analysis then shows the numerical trajectory is the exact trajectory of a nearby *modified* energy $\tilde{H} = H + O(h^p)$, so the true energy oscillates around a constant instead of drifting.

### Unit Oscillator

For $a = -\omega^2 q$ and $\theta = h\omega$, both methods have a step matrix with determinant $1$ and trace $2 - \theta^2$. The eigenvalues stay on the unit circle, hence the trajectory is bounded, exactly when $|2 - \theta^2| < 2$, i.e.

$$
h\,\omega < 2 .
$$

The conserved modified energies and the resulting error of the true energy $E = \tfrac12(v^2 + \omega^2 q^2)$ are:

| Method | Conserved quantity ($\omega = 1$) | Relative energy error |
|--------|-----------------------------------|-----------------------|
| Semi-implicit Euler | $\tfrac12\left(v^2 + q^2 - h\,q\,v\right)$ | oscillates, amplitude $\approx h\omega/2$ |
| Velocity Verlet | $\tfrac12\left(v^2 + (1 - \tfrac{\theta^2}{4})\,q^2\right)$ | oscillates, amplitude $\approx (h\omega)^2/8$ |

### Velocity-Dependent Forces

With damping or Coriolis terms the system is not Hamiltonian, so neither method is exactly symplectic. Verlet evaluates the second kick at the half-step velocity $v_{1/2}$, which keeps second-order accuracy. Semi-implicit Euler evaluates its single acceleration at the old velocity. Energy decay is then physical dissipation, not a numerical artifact.

## Complexity Analysis

| Method | Acceleration evaluations | Order | Arithmetic | Memory |
|--------|--------------------------|-------|------------|--------|
| Semi-implicit Euler | 1 | 1 | $O(n)$ | two state copies |
| Velocity Verlet | 2 | 2 | $O(n)$ | two state copies |

Here $n$ is the number of generalized coordinates. The cost of a step is the cost of its evaluations plus a handful of vector operations; no heap is used. Each step is a pure function of its arguments; the second Verlet evaluation is not reused as the next step's first. Every evaluation goes through a virtual call: fine for simulation and model-in-the-loop tests; for interrupt-service use, bind the concrete system type instead.

## Step-by-Step Walkthrough

Unit oscillator $a = -q$, $h = 0.1$, start $(q, v) = (1, 0)$, true energy $E = 0.5$.

**Semi-implicit Euler**

| Step | Velocity update | Position update | True $E$ | Modified energy |
|------|-----------------|-----------------|----------|-----------------|
| 1 | $0 + 0.1(-1) = -0.1$ | $1 + 0.1(-0.1) = 0.99$ | $0.4950$ | $0.5000$ |
| 2 | $-0.1 + 0.1(-0.99) = -0.199$ | $0.99 + 0.1(-0.199) = 0.9701$ | $0.4903$ | $0.5000$ |

The true energy wanders (about $1\%$ and $2\%$ low), yet the modified energy is conserved exactly; the offset is $\tfrac12 h\,q\,v$, which cannot grow beyond $h/4$.

**Velocity Verlet**, one step: half kick $v_{1/2} = 0 + 0.05(-1) = -0.05$; drift $q^+ = 1 + 0.1(-0.05) = 0.995$; half kick with the new position $v^+ = -0.05 + 0.05(-0.995) = -0.09975$. Exact values are $\cos 0.1 = 0.995004$ and $-\sin 0.1 = -0.099833$. The true energy is $0.49999$, and the modified energy $\tfrac12\left(v^2 + 0.9975\,q^2\right) = 0.49875$ equals its initial value.

## Pitfalls & Edge Cases

- **Step size.** Both methods require $h\omega < 2$ for the fastest mode; beyond that the iteration diverges. The step size must be positive, and a non-positive value is rejected at construction.
- **Not for stiff or dissipative-dominated plants.** Both methods are explicit. Strong damping or stiffness needs an implicit method or a much smaller step.
- **Conservative forces only for the guarantee.** Velocity-dependent forces and discontinuities (contact, saturation, friction) break symplecticity; the energy bound is lost, though the order of accuracy remains.
- **Single-precision floor.** Rounding bounds the achievable accuracy. At $h = 0.01$ the Verlet error ratio between $h$ and $h/2$ departs from the ideal $4$, while at $h = 0.1$ versus $0.05$ it is $4.00$. Halving the step below the rounding floor buys nothing.
- **Measured energy error.** Over $10^5$ steps at $h = 0.01$ on the unit oscillator, the maximum relative energy error is $0.51\%$ for semi-implicit Euler (matching $h\omega/2$) and $4\times10^{-5}$ for Verlet, of the same order as $(h\omega)^2/8$ with a single-precision rounding contribution.

## Variants & Generalizations

| Variant | Key difference |
|---------|----------------|
| **Semi-implicit Euler, velocity Verlet (this)** | Symplectic, orders 1 and 2; explicit Euler differs only in using the old velocity for the drift and its energy grows |
| **Higher-order composition** | Chains of Verlet steps with weights (Yoshida, Suzuki) reach orders 4, 6, ... |
| **Implicit midpoint / Gauss** | Symplectic and implicit; suited to stiff or non-separable systems at the cost of a nonlinear solve |

## Applications

- **Plant simulation** — long model-in-the-loop runs of mechanical plants, where numerical energy drift would corrupt stability margins.
- **Lightly damped dynamics** — pendulums, flexible links, motor plus elastic load, whose true damping must not be masked by numerical damping.

## Connections to Other Algorithms

| Algorithm | Relationship |
|-----------|--------------|
| Runge–Kutta 4 | Four evaluations per step, fourth order, general first-order systems; energy drifts slowly on conservative problems |
| Dormand–Prince | Adaptive step and error control for first-order systems; higher accuracy per step, no structure preservation |

## References & Further Reading

- Hairer, E., Lubich, C., and Wanner, G., "Geometric numerical integration illustrated by the Störmer–Verlet method," *Acta Numerica*, 12:399–450, 2003.
- Hairer, E., Lubich, C., and Wanner, G., *Geometric Numerical Integration*, 2nd ed., Springer, 2006 — Chapters I and VI.
- Verlet, L., "Computer experiments on classical fluids," *Physical Review*, 159(1):98–103, 1967.

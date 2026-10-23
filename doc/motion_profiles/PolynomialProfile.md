# Polynomial Point-to-Point Profile

## Overview & Motivation

A polynomial profile is a closed-form time law that drives one axis from a start state to a goal state over a fixed duration $T$. A **cubic** matches endpoint position and velocity; a **quintic** additionally matches endpoint acceleration, so the motion has no acceleration step at either end.

Point-to-point moves are the bread and butter of servo motion: robot joints, CNC and pick-and-place axes, motor position loops. The coefficients come from a handful of algebraic expressions, solved once. Each servo tick is then a fixed number of multiply-adds, with no iteration, no lookup table and a deterministic cycle count.

## Mathematical Theory

### Boundary Conditions

Let $t \in [0, T]$, $\Delta q = q_f - q_0$, and let $\alpha_0, \alpha_f$ be the start and end accelerations. A polynomial of degree $n$ has $n+1$ coefficients, so a cubic satisfies four conditions and a quintic six:

$$q(0) = q_0,\quad q(T) = q_f,\quad \dot{q}(0) = v_0,\quad \dot{q}(T) = v_f \qquad \text{(quintic also: } \ddot{q}(0) = \alpha_0,\ \ddot{q}(T) = \alpha_f\text{)}$$

The conditions are linear in the coefficients $a_k$ of $q(t) = \sum_k a_k t^k$ and the time basis is fixed, so the solution is a plug-in formula rather than a runtime matrix solve.

### Cubic

The start conditions give $a_0 = q_0$ and $a_1 = v_0$. The end conditions leave a $2\times 2$ system with the closed-form solution

$$a_2 = \frac{3\,\Delta q - (2 v_0 + v_f)\,T}{T^2}, \qquad a_3 = \frac{-2\,\Delta q + (v_0 + v_f)\,T}{T^3}$$

The acceleration is linear in $t$ and the jerk is the constant $6 a_3$.

### Quintic

The start conditions give $a_0 = q_0$, $a_1 = v_0$, $a_2 = \alpha_0 / 2$. The end conditions give

$$a_3 = \frac{20\,\Delta q - (8 v_f + 12 v_0)\,T - (3\alpha_0 - \alpha_f)\,T^2}{2\,T^3}$$

$$a_4 = \frac{-30\,\Delta q + (14 v_f + 16 v_0)\,T + (3\alpha_0 - 2\alpha_f)\,T^2}{2\,T^4}$$

$$a_5 = \frac{12\,\Delta q - 6 (v_f + v_0)\,T - (\alpha_0 - \alpha_f)\,T^2}{2\,T^5}$$

The acceleration is a cubic matching $\alpha_0$ and $\alpha_f$, hence continuous; the jerk is a quadratic.

### Sampling

Position, velocity, acceleration and jerk are four Horner evaluations of the polynomial and its three derivatives:

$$q = a_0 + t\big(a_1 + t(a_2 + t(a_3 + t(a_4 + t\,a_5)))\big), \qquad \dddot{q} = 6 a_3 + t\,(24 a_4 + 60 a_5\, t)$$

The cubic is the same evaluation with $a_4 = a_5 = 0$. The time is first clamped to $[0, T]$.

### Rest-to-Rest Peaks

With zero end velocities (and accelerations, for the quintic) the cubic peaks at $1.5\,\Delta q / T$ in velocity and $6\,\Delta q / T^2$ in acceleration; the quintic at $1.875\,\Delta q / T$ and $\tfrac{10}{\sqrt{3}}\,\Delta q / T^2 \approx 5.77\,\Delta q / T^2$.

## Complexity Analysis

| Operation         | Time               | Space  | Notes                                                                                |
|-------------------|--------------------|--------|--------------------------------------------------------------------------------------|
| Coefficient solve | $O(1)$             | $O(1)$ | Closed-form expressions, once at construction or reset                               |
| Sample            | $O(\text{degree})$ | $O(1)$ | Four Horner evaluations, at most 14 multiply-adds; no state change, so deterministic |
| Stored state      | n/a                | $O(1)$ | Six coefficients, the duration and the degree; stack only, no heap                   |

## Step-by-Step Walkthrough

Rest-to-rest cubic with $q_0 = 0$, $q_f = 1$, $T = 2$, $v_0 = v_f = 0$, so $\Delta q = 1$ and $a_0 = a_1 = 0$.

| Step         | Computation                                   | Result                                       |
|--------------|-----------------------------------------------|----------------------------------------------|
| $a_2$        | $(3\cdot 1 - 0)/2^2$                          | $0.75$                                       |
| $a_3$        | $(-2\cdot 1 + 0)/2^3$                         | $-0.25$                                      |
| Position     | $0.75\,t^2 - 0.25\,t^3 = 3(t/2)^2 - 2(t/2)^3$ | $q(1) = 0.5$, $q(2) = 1$                     |
| Velocity     | $1.5\,t - 0.75\,t^2$                          | $v(1) = 0.75 = 1.5\,\Delta q / T$ (the peak) |
| Acceleration | $1.5 - 1.5\,t$                                | $+1.5$ at $t=0$, $-1.5$ at $t=2$             |
| Jerk         | $6 a_3$                                       | $-1.5$, constant                             |

The position is the smoothstep of $t/2$, symmetric about the midpoint, where the velocity peaks. The acceleration jumps from $0$ at rest to $1.5$ at the first instant and from $-1.5$ back to $0$ at the end.

## Pitfalls & Edge Cases

- **Duration must be positive.** The coefficients contain $1/T^k$ terms; a zero or negative duration is rejected at construction and reset.
- **Short durations in single precision.** Higher-order coefficients grow like $1/T^5$ for the quintic, and terms of opposite sign cancel near the end of the move, losing digits.
- **Clamped time holds the endpoint state.** Outside $[0, T]$ the sample is the state at $0$ or $T$, so it reports the end velocity and acceleration, not zero, when they are nonzero.
- **Acceleration steps in the cubic.** Acceleration is not matched, so at rest endpoints it jumps from or to zero, which excites flexible loads. The quintic's jerk is finite but generally nonzero at the ends.
- **No limit enforcement.** Peaks follow from $\Delta q$, $T$ and the boundary values; inconsistent boundary velocities can overshoot $q_f$ before returning.

## Variants & Generalizations

| Variant                     | Key difference                                                                                                            |
|-----------------------------|---------------------------------------------------------------------------------------------------------------------------|
| **Cubic (this)**            | Position and velocity matched; constant jerk; acceleration discontinuous at the ends                                      |
| **Quintic (this)**          | Adds acceleration matching; continuous acceleration at about $25\%$ higher peak velocity for rest-to-rest                 |
| **Path-parameter time law** | A 0 to 1 instance maps time to a normalized path coordinate; with $v_0 = v_f = 1$, $T = 1$ the cubic collapses to $s = t$ |

## Applications

- **Joint-space moves** — robot joints and servo axes between taught points; multi-axis moves use one instance per axis with a common $T$ so all axes arrive together.
- **Pick-and-place and CNC positioning** — fixed-time axis moves where smoothness matters more than minimum time.
- **Path timing** — the scalar time law along a geometric path such as an orientation interpolation.

## Connections to Other Algorithms

| Algorithm                    | Relationship                                                                                                          |
|------------------------------|-----------------------------------------------------------------------------------------------------------------------|
| Trapezoidal velocity profile | Respects velocity and acceleration limits with a free duration; polynomials fix the duration and let the peaks follow |
| S-curve profile              | Bounds jerk explicitly under limits; the quintic only smooths the endpoints                                           |
| Cubic spline interpolation   | Handles many via points with continuous acceleration; for two knots it reduces to this cubic                          |

## References & Further Reading

- Craig, J. J., "Introduction to Robotics: Mechanics and Control," 3rd ed., Pearson, 2005, Chapter 7 (trajectory generation).
- Biagiotti, L. and Melchiorri, C., "Trajectory Planning for Automatic Machines and Robots," Springer, 2008, §2.1.

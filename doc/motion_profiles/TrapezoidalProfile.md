# Trapezoidal Profile (Linear Segment with Parabolic Blends)

## Overview & Motivation

A point-to-point move has to reach its goal as fast as the actuator allows without saturating the drive. The trapezoidal profile — *linear segment with parabolic blends* (LSPB) — does this with the simplest rule that respects both a velocity ceiling and an acceleration ceiling: accelerate at the maximum rate, coast at the maximum speed, decelerate at the maximum rate. Velocity plotted against time is a trapezoid; a move too short to reach the ceiling becomes a triangle.

From the limits and the distance alone, the plan is the **minimum-time** one. A second mode fixes the duration instead, which multi-axis synchronization needs: all axes must finish together, so the faster ones are slowed rather than the slowest sped up.

## Mathematical Theory

### Problem Statement

Rest-to-rest move of length $d = |q_f - q_0|$ in direction $\sigma = \pm 1$ (positive when $q_f \ge q_0$), with limits $v_{max} > 0$ and $a_{max} > 0$. The profile is planned for the magnitude and mirrored by $\sigma$.

### Minimum-Time Plan

A ramp between rest and speed $v$ takes $v/a_{max}$ and covers $v^2/(2a_{max})$, so the two ramps together consume the **blend distance** $v^2/a_{max}$. Comparing $d$ with the blend distance at $v_{max}$ selects the shape:

$$
\begin{cases}
d \ge v_{max}^2/a_{max}: & v = v_{max}, \quad t_a = v/a_{max}, \quad t_c = (d - v^2/a_{max})/v \quad\text{(trapezoid)} \\
d < v_{max}^2/a_{max}: & v = \sqrt{d\,a_{max}}, \quad t_a = v/a_{max}, \quad t_c = 0 \quad\text{(triangle)}
\end{cases}
$$

The total time is $T = 2t_a + t_c$. The cases agree at $d = v_{max}^2/a_{max}$, so $T$ is continuous in $d$.

### Sampling

With $v$, $t_a$, $t_c$, $T$ precomputed, the unsigned state is:

| Phase | Interval | Acceleration | Velocity | Position |
|-------|----------|--------------|----------|----------|
| Parabolic ramp-up | $0 \le t < t_a$ | $a_{max}$ | $a_{max}\,t$ | $\tfrac12 a_{max}\,t^2$ |
| Linear cruise | $t_a \le t < t_a + t_c$ | $0$ | $v$ | $v\,(t - t_a/2)$ |
| Parabolic ramp-down | $t_a + t_c \le t < T$ | $-a_{max}$ | $a_{max}\,(T-t)$ | $d - \tfrac12 a_{max}\,(T-t)^2$ |

Velocity, acceleration and the position offset are multiplied by $\sigma$. At and beyond $t = T$ the profile holds the goal at rest (velocity and acceleration zero); times before $0$ are treated as $t = 0$, holding the start with zero velocity. The reported jerk is zero in every phase.

### Fixed-Duration Plan

The acceleration $a_{max}$ is kept and the cruise speed is lowered until the move takes exactly $T$. The ramps cover $v\,t_a$ in total and the cruise covers $v\,(T - 2t_a)$, so with $t_a = v/a_{max}$

$$
d = v\left(T - \frac{v}{a_{max}}\right) \;\Longleftrightarrow\; v^2 - a_{max}T\,v + a_{max}\,d = 0 .
$$

The smaller root keeps $t_c \ge 0$. Multiplying it by its conjugate gives a form with no subtraction of nearly equal numbers:

$$
v = \frac{a_{max}T - \sqrt{a_{max}^2T^2 - 4a_{max}d}}{2} = \frac{2\,a_{max}\,d}{a_{max}T + \sqrt{a_{max}^2T^2 - 4a_{max}d}} .
$$

A real root exists iff $a_{max}\,T^2 \ge 4d$. Equality gives a double root $v = a_{max}T/2$ and a triangle: $T$ is then the shortest duration achievable at this acceleration. Otherwise $t_a = v/a_{max}$ and $t_c = T - 2t_a$.

### Multi-Axis Synchronization

Let $T_{sync}$ be the longest axis time and $\lambda = T_{sync}/T_{axis} \ge 1$. Planning an axis again with $(v_{max}/\lambda,\; a_{max}/\lambda^2)$ leaves the blend distance $v_{max}^2/a_{max}$ unchanged, so the same shape is selected, while $t_a$ and $t_c$ both scale by $\lambda$: the duration is exactly $\lambda\,T_{axis}$. Velocity is divided by $\lambda$ and acceleration by $\lambda^2$.

The fixed-duration plan is the alternative. It keeps $a_{max}$ and only lowers the cruise speed: the smaller root decreases as $T$ grows, so for $T \ge T_{min}$ the cruise speed never exceeds $v_{max}$.

## Complexity Analysis

| Operation | Time | Space | Notes |
|-----------|------|-------|-------|
| Minimum-time plan | $O(1)$ | $O(1)$ | One comparison, one square root (triangle only) |
| Fixed-duration plan | $O(1)$ | $O(1)$ | One square root, one division |
| Sample | $O(1)$ | $O(1)$ | One phase branch, a few multiply-adds; no state change |

Memory is a fixed handful of scalars: start, direction, distance, acceleration, peak speed and three phase times.

## Step-by-Step Walkthrough

**Trapezoid.** $d = 5$, $v_{max} = 1$, $a_{max} = 2$. The blend distance is $1/2 = 0.5 \le 5$, so a cruise exists: $t_a = 0.5$, $t_c = (5 - 0.5)/1 = 4.5$, $T = 2(0.5) + 4.5 = 5.5$.

| $t$ | Phase | Position | Velocity | Acceleration |
|-----|-------|----------|----------|--------------|
| $0.25$ | ramp-up | $\tfrac12\cdot 2\cdot 0.25^2 = 0.0625$ | $0.5$ | $2$ |
| $3$ | cruise | $1\cdot(3 - 0.25) = 2.75$ | $1$ | $0$ |
| $5.25$ | ramp-down | $5 - \tfrac12\cdot 2\cdot 0.25^2 = 4.9375$ | $0.5$ | $-2$ |
| $\ge 5.5$ | held | $5$ | $0$ | $0$ |

The ramp-up ends at position $0.25$, and the cruise formula gives the same value at $t = 0.5$. The triangle threshold for these limits is $d = 0.5$; for $d = 0.25$, $v = \sqrt{0.5} \approx 0.7071$ and $T \approx 0.7071$, with no cruise.

**Fixed duration.** $d = 4$, $a_{max} = 1$, $T = 5$: discriminant $25 - 16 = 9$, $v = 2\cdot 4/(5 + 3) = 1$, $t_a = 1$, $t_c = 3$; check $1\cdot(5 - 1) = 4$. For $d = 5$, $a_{max} = 2$, $T = 6$: discriminant $144 - 40 = 104$, $v = 20/(12 + \sqrt{104}) \approx 0.900980$.

## Pitfalls & Edge Cases

- **Non-positive limits.** The minimum-time plan requires $v_{max} > 0$ and $a_{max} > 0$; a violation is an assertion failure, not a recoverable status. The fixed-duration plan returns no result for $a_{max} \le 0$ or $T \le 0$.
- **Infeasible duration.** If $a_{max}T^2 < 4d$ the plan is rejected, never clamped; the shortest feasible duration is $2\sqrt{d/a_{max}}$. At the boundary, single-precision rounding can push the discriminant just below zero, so plan the slowest axis with the minimum-time plan.
- **Cancellation.** The textbook root $(a_{max}T - \sqrt{\cdot})/2$ loses its significant digits in single precision when $4a_{max}d \ll a_{max}^2T^2$; the conjugate form does not. The computed cruise time is clamped at zero.
- **Zero distance.** The minimum-time plan has $v = 0$ and $T = 0$: a zero-length profile that is at rest at the goal for every $t \ge 0$.
- **Discontinuous acceleration.** Position and velocity are continuous, but acceleration jumps between $0$ and $\pm a_{max}$ at the joins. The jerk impulses are not represented in the reported jerk and can excite structural modes; use the S-curve profile in that case.

## Variants & Generalizations

The **S-curve** profile adds a jerk limit (seven segments) for continuous acceleration at the cost of a longer move. **Polynomial** profiles (cubic, quintic) are fixed-duration and smooth by construction but enforce no velocity or acceleration ceiling. Unequal acceleration and deceleration limits use the same construction with unequal ramp times.

## Applications

- **Point-to-point axis moves** — servo and stepper drives, gantries, pick-and-place, gimbal slews.
- **Multi-axis coordination** — time-scaled or fixed-duration plans make every axis arrive together.

## Connections to Other Algorithms

The S-curve profile is the jerk-limited upgrade under the same velocity and acceleration limits. The polynomial profile is fixed-time and smooth but offers no limit guarantees. The saturation rate limiter slews a changing setpoint online, whereas the trapezoid plans a whole move offline.

## References & Further Reading

- Biagiotti, L. and Melchiorri, C., "Trajectory Planning for Automatic Machines and Robots," Springer, 2008, §3.2.
- Siciliano, B., Sciavicco, L., Villani, L., and Oriolo, G., "Robotics: Modelling, Planning and Control," Springer, 2009, §4.2.

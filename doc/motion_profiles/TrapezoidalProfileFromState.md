# Trapezoidal Profile from a Moving State

## Overview & Motivation

A rest-to-rest trapezoid ([Trapezoidal Profile](TrapezoidalProfile.md)) assumes the axis is still when the move begins. A servo that takes a new goal while it is already moving cannot use it: planning from rest would step the reference velocity to zero and back, which is exactly the jolt a profile exists to avoid. This profile plans from the current position **and velocity**, with separate limits for speeding up and slowing down, and still reaches the goal at rest in minimum time. A new goal can therefore be accepted at any instant: the plan starts where the previous one is, and the reference stays continuous in position and velocity.

The construction follows the classic one-dimensional time-optimal plan with an initial velocity, the one ODrive's trajectory planner uses, with the slowing-down limit applied wherever the speed magnitude falls.

## Mathematical Theory

### Problem Statement

Start at $q_0$ with velocity $v_0$; reach $q_f$ at rest. Limits: cruise speed $v_{max} > 0$, acceleration $a_{max} > 0$ (speed magnitude rising) and deceleration $d_{max} > 0$ (speed magnitude falling). Let $\Delta = q_f - q_0$.

### Direction of the Cruise

Stopping from $v_0$ at $d_{max}$ travels the signed distance $s_{stop} = v_0\,|v_0| / (2d_{max})$. If the goal lies beyond that point the cruise moves forward, otherwise backward:

$$
\sigma = \begin{cases} +1 & \Delta - s_{stop} \ge 0 \\ -1 & \text{otherwise} \end{cases}
$$

A start moving away from the goal, or one too fast to stop before it, therefore first decelerates through zero and cruises back: the overshoot is part of the plan, not an error.

### Three Phases

1. **Ramp** from $v_0$ to the cruise velocity $v_c = \sigma v_{max}$ at rate $a_r$. Normally $a_r = \sigma a_{max}$. If the axis already moves faster than $v_{max}$ in the cruise direction ($\sigma v_0 > v_{max}$) the ramp slows it, so $a_r = -\sigma d_{max}$. The ramp lasts $t_a = (v_c - v_0)/a_r \ge 0$.
2. **Cruise** at $v_c$ for $t_c$.
3. **Deceleration** to rest at $-\sigma d_{max}$, lasting $t_d = v_{max}/d_{max}$.

Ramp and deceleration alone cover

$$
\Delta_{min} = \tfrac12 t_a (v_c + v_0) + \tfrac12 t_d\, v_c .
$$

If $\sigma\Delta \ge \sigma\Delta_{min}$ the cruise takes the rest: $t_c = (\Delta - \Delta_{min})/v_c$.

### Short Moves

Otherwise the velocity never reaches $v_{max}$ and peaks at $v_p$. The ramp at $\sigma a_{max}$ and the deceleration at $\sigma d_{max}$ together cover $\Delta$:

$$
\frac{v_p^2 - v_0^2}{2\sigma a_{max}} + \frac{v_p^2}{2\sigma d_{max}} = \Delta
\;\Longrightarrow\;
v_p = \sigma\sqrt{\max\!\left(0,\; \frac{2\sigma\Delta\, a_{max} d_{max} + v_0^2\, d_{max}}{a_{max} + d_{max}}\right)}
$$

with $t_a = \max(0, (v_p - v_0)/a_r)$, $t_d = |v_p|/d_{max}$, $t_c = 0$. A start above $v_{max}$ never reaches this case: with $a_r = -\sigma d_{max}$ the minimum distance equals the stopping distance, which the choice of $\sigma$ already guarantees.

### Sampling

With $T = t_a + t_c + t_d$ and the cruise velocity $v$ ($v_c$, or $v_p$ for a short move):

| Phase        | Interval                | Acceleration      | Velocity                  | Position                                      |
|--------------|-------------------------|-------------------|---------------------------|-----------------------------------------------|
| Ramp         | $0 \le t < t_a$         | $a_r$             | $v_0 + a_r t$             | $q_0 + v_0 t + \tfrac12 a_r t^2$              |
| Cruise       | $t_a \le t < t_a + t_c$ | $0$               | $v$                       | $q_0 + \tfrac12 t_a (v + v_0) + v\,(t - t_a)$ |
| Deceleration | $t_a + t_c \le t < T$   | $-\sigma d_{max}$ | $\sigma d_{max}\,(T - t)$ | $q_f - \tfrac12 \sigma d_{max}\,(T - t)^2$    |

Beyond $T$ the profile holds $q_f$ at rest; times before $0$ are treated as $t = 0$. The deceleration phase is anchored to the goal, so the final position is exact whatever rounding the earlier phases carry. The reported jerk is zero.

## Complexity Analysis

| Operation | Time   | Space  | Notes                                                     |
|-----------|--------|--------|-----------------------------------------------------------|
| Plan      | $O(1)$ | $O(1)$ | A few divisions; one square root for a short move         |
| Sample    | $O(1)$ | $O(1)$ | One phase branch and a few multiply-adds; no state change |

No loop or iteration: the cost is fixed, which suits a replan inside a control interrupt.

## Step-by-Step Walkthrough

**From rest.** $q_0 = 0$, $v_0 = 0$, $q_f = 1.5$, $v_{max} = 20$, $a_{max} = d_{max} = 400$. $s_{stop} = 0$, so $\sigma = +1$. $t_a = 20/400 = 0.05$, $t_d = 0.05$, $\Delta_{min} = 0.5 + 0.5 = 1$, $t_c = 0.5/20 = 0.025$, $T = 0.125$ — the rest-to-rest trapezoid.

**Moving away.** Same goal, $v_0 = -10$. $s_{stop} = -0.125$, $\Delta - s_{stop} > 0$, so $\sigma = +1$. $t_a = 30/400 = 0.075$, covering $-0.75 + 1.125 = 0.375$; the deceleration covers $0.5$; $t_c = (1.5 - 0.875)/20 = 0.03125$; $T = 0.15625$.

**Overshoot.** $q_f = 0.2$, $v_0 = 20$, $d_{max} = 400$. $s_{stop} = 0.5 > 0.2$, so $\sigma = -1$: the axis decelerates through zero, cruises back and settles on $0.2$ from above.

## Pitfalls & Edge Cases

- **Zero or negative limits** make the phase times undefined; they are preconditions.
- **Goal at the stopping point** ($\Delta = s_{stop}$) gives $\sigma = +1$ and a pure deceleration: $t_a$ brings the velocity to $v_{max}$ only if it is below it; for $v_0 = 0$ and $\Delta = 0$ the plan is empty, $T = 0$.
- **Rounding near phase boundaries** can leave a few micro-units of position jump between cruise and deceleration; anchoring the last phase to $q_f$ keeps the goal exact.
- **Acceleration is discontinuous**, as in every trapezoid. Where jerk matters, use the [S-Curve Profile](SCurveProfile.md).

## Variants & Generalizations

- **Symmetric limits** ($a_{max} = d_{max}$) and $v_0 = 0$ reduce exactly to the rest-to-rest [Trapezoidal Profile](TrapezoidalProfile.md).
- **Non-zero final velocity** follows by shifting the deceleration target; not needed for point-to-point moves.
- **Jerk limits** turn each ramp into three phases (S-curve); starting from a non-zero acceleration then needs a ramp of the acceleration to zero first.

## Applications

- Position servos that accept a new setpoint mid-move (CiA 402 profile position mode with "change set immediately").
- Joint-space moves that are retargeted by a supervisor.

## Connections to Other Algorithms

- [Trapezoidal Profile](TrapezoidalProfile.md): the rest-to-rest special case, plus fixed-duration planning.
- [S-Curve Profile](SCurveProfile.md): continuous acceleration at the cost of a longer move.

## References & Further Reading

- L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer, 2008, §3.2 (trapezoidal velocity profile with initial and final velocities).
- ODrive firmware, `trapTraj.cpp`, `planTrapezoidal`: the same direction choice and short-move formula.

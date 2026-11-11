# S-Curve Profile from a Moving State

## Overview & Motivation

The rest-to-rest [S-Curve Profile](SCurveProfile.md) bounds jerk, but assumes the axis is still when the move begins. The [Trapezoidal Profile from a Moving State](TrapezoidalProfileFromState.md) accepts a new goal mid-move, but its acceleration jumps. A servo that replans while its reference is in a jerk or acceleration phase needs both: a plan that starts from the current position, velocity **and acceleration**, keeps all three continuous across the replan, bounds jerk, speed and acceleration, and still ends on the goal at rest with zero acceleration.

No compact closed form covers every start: a non-zero initial acceleration, a start moving away from the goal, a start above the speed limit and asymmetric acceleration limits multiply the cases. This profile keeps the seven-phase structure and finds its one free parameter — the peak velocity — with a fixed-count bisection. Planning runs only on a setpoint change; sampling stays a loop-free evaluation of one cubic.

## Mathematical Theory

### Problem Statement

Start at $q_0$ with velocity $v_0$ and acceleration $a_0$; reach $q_f$ with $v = a = 0$. Limits: speed $v_{max}$, acceleration $a_{max}$ while the speed magnitude rises, deceleration $d_{max}$ while it falls, jerk $j_{max}$, all positive, and $|a_0| \le \max(a_{max}, d_{max})$. Let $\Delta = q_f - q_0$.

### Velocity Change with Zero Final Acceleration

The building block takes the state $(v_s, a_s)$ to $(v_t, 0)$ in at most three constant-jerk segments: a jerk segment to a plateau acceleration $\pm A$, a hold at $\pm A$, and a jerk segment back to zero.

Bringing $a_s$ straight to zero reaches $v_z = v_s + a_s|a_s| / (2j_{max})$. The plateau sign is $s = +1$ if $v_t \ge v_z$, else $-1$. Without a hold, the change $s(v_t - v_s) = (2A^2 - a_s^2)/(2j_{max})$ gives

$$
A_{free} = \sqrt{\max\!\left(0,\; \frac{2 j_{max}\, s(v_t - v_s) + a_s^2}{2}\right)}, \qquad A = \min(A_{free}, L).
$$

The choice of $s$ guarantees $A_{free} \ge s\,a_s$, so the formula never asks the first segment to go the wrong way. The segment durations are

$$
t_1 = \frac{|sA - a_s|}{j_{max}}, \qquad t_3 = \frac{A}{j_{max}}, \qquad
t_2 = \max\!\left(0,\; \frac{v_t - v_s - \tfrac12 (a_s + sA)\,t_1 - \tfrac12 sA\, t_3}{sA}\right),
$$

with jerk $\operatorname{sign}(sA - a_s)\,j_{max}$ in the first and $-s\,j_{max}$ in the last. If $|a_s|$ already exceeds $L$ in the plateau direction, $A_{free} \ge |a_s| > L$, so the first segment brings the acceleration down to the limit.

The plateau limit $L$ follows the speed magnitude: $d_{max}$ if the velocity moves towards zero without crossing it ($s v_s \le 0$ and $s v_t \le 0$), $a_{max}$ if it moves away from zero ($s v_s \ge 0$ and $s v_t \ge 0$), and $\min(a_{max}, d_{max})$ for a change that crosses zero, whose single plateau both slows and speeds up the axis.

### Direction

The minimum stopping distance $s_{stop}$ is the distance of the velocity change $(v_0, a_0) \to (0, 0)$. As in the trapezoidal planner, the plan moves forward if $\Delta - s_{stop} \ge 0$ and backward otherwise: $\sigma = \pm 1$. All further planning uses the frame $v'_0 = \sigma v_0$, $a'_0 = \sigma a_0$, $\Delta' = \sigma \Delta \ge s'_{stop}$. A start moving away from the goal, or one too fast to stop before it, therefore turns round and comes back.

### Seven Phases and the Peak Velocity

In that frame the plan is

1. a velocity change $(v'_0, a'_0) \to (v_p, 0)$ — three segments;
2. a cruise at $v_p$ for $t_c$;
3. a velocity change $(v_p, 0) \to (0, 0)$ at $d_{max}$ — three segments.

Let $f(v_p)$ be the distance phases 1 and 3 cover. If $f(v_{max}) \le \Delta'$ then $v_p = v_{max}$ and $t_c = (\Delta' - f(v_{max}))/v_{max}$; a start above $v_{max}$ is brought down to it at $d_{max}$. Otherwise there is no cruise and $v_p \in [0, v_{max}]$ solves $f(v_p) = \Delta'$.

$f(0) = s'_{stop} \le \Delta'$ by the choice of $\sigma$, so a root exists. Above $v'_z = \operatorname{clamp}(v'_0 + a'_0|a'_0|/(2j_{max}),\, 0,\, v_{max})$ phase 1 raises the velocity and $f$ increases with $v_p$. Below it, phase 1 already slows the axis and $f$ need not be monotonic. The bracket is therefore $[v'_z, v_{max}]$ if $f(v'_z) \le \Delta'$, and $[0, v'_z]$ otherwise; in both, the lower end covers at most $\Delta'$ and the upper end more, which is all bisection needs to converge on a root.

The bisection runs **32 iterations**, a fixed count: for single precision the bracket reaches adjacent floats after about 24. The lower end is kept, so the plan never covers more than $\Delta'$; the remainder (a rounding residue) is added as cruise time $(\Delta' - f(v_p))/v_p$.

### Sampling

Integrating the seven segments forward from $(q_0, v_0, a_0)$ stores, for each segment, its start time and the position, velocity and acceleration there, plus its jerk, all mapped back by $\sigma$. A sample at $t$ picks the segment with a three-level comparison tree on the start times — no loop — and evaluates

$$
q = q_k + v_k\tau + \tfrac12 a_k \tau^2 + \tfrac16 j_k \tau^3,\quad v = v_k + a_k\tau + \tfrac12 j_k\tau^2,\quad a = a_k + j_k\tau,\qquad \tau = t - t_k .
$$

Beyond the end the profile holds $q_f$ at rest; times before $0$ are treated as $t = 0$.

## Complexity Analysis

| Operation | Time   | Space  | Notes                                                                                                |
|-----------|--------|--------|------------------------------------------------------------------------------------------------------|
| Plan      | $O(1)$ | $O(1)$ | At most $3 + 32$ evaluations of $f$, each two velocity changes: about 70 square roots, 300 divisions |
| Sample    | $O(1)$ | $O(1)$ | Three comparisons, one cubic; no loop, no state change                                               |

Plan cost is bounded but two orders of magnitude above a sample: plan on a setpoint change, outside the fastest loop if possible, and sample every tick. The stored plan is about 40 values.

## Step-by-Step Walkthrough

**From rest.** $q_0 = 0$, $v_0 = a_0 = 0$, $q_f = 1.5$, $v_{max} = 20$, $a_{max} = d_{max} = 400$, $j_{max} = 20000$. $s_{stop} = 0$, $\sigma = +1$. For $v_p = 20$: $A_{free} = \sqrt{400000} > 400$, so $A = 400$, $t_1 = t_3 = 0.02$, each covering $4$ of velocity, $t_2 = 12/400 = 0.03$; phase 1 lasts $0.07$ and covers $20 \cdot 0.07/2 = 0.7$. Phase 3 mirrors it. $f(20) = 1.4 \le 1.5$, so $t_c = 0.1/20 = 0.005$ and $T = 0.145$ — the symmetric double-S value.

**Separate deceleration.** Same, $q_f = 2$, $d_{max} = 200$. Phase 1: $0.07$, $0.7$. Phase 3: $t_1 = t_3 = 0.01$, $t_2 = 18/200 = 0.09$, $0.11$ in all, covering $1.1$. $t_c = 0.2/20 = 0.01$, $T = 0.19$.

**Start in an acceleration phase.** $v_0 = 15$, $a_0 = 300$, $q_f = 0.4$. $v_z = 15 + 300^2/40000 = 17.25$: the axis cannot stop before reaching $17.25$ and the stopping distance exceeds $0.4$, so $\sigma = -1$. The first segment ramps the acceleration down at $-j_{max}$ through zero to $-d_{max}$; the axis overshoots and settles on $0.4$ from above.

**Large jerk.** $v_0 = 10$, $q_f = 1.5$, $a_{max} = 400$, $d_{max} = 250$, $j_{max} = 10^7$: the duration is within $10^{-4}$ of the trapezoid's $0.12125$.

## Pitfalls & Edge Cases

- **Zero or negative limits** and $|a_0| > \max(a_{max}, d_{max})$ are preconditions.
- **An initial acceleration pushing past $v_{max}$** ($v'_z > v_{max}$) cannot be undone instantly: the speed exceeds $v_{max}$ for the time the jerk needs to bring $a_0$ to zero, then the plan brings it down.
- **Near-time-optimal, not optimal.** When $f(v'_z) > \Delta'$, the plan slows to $v_p < v'_z$ with zero acceleration for an instant, then stops. It reaches the goal within the limits, but a plan without that notch can be slightly shorter.
- **A zero crossing is limited by $\min(a_{max}, d_{max})$**, where the trapezoidal planner uses $a_{max}$. With $d_{max} < a_{max}$ a reversal is longer than the trapezoid even for very large jerk; with symmetric limits the two agree.
- **The goal is reached by forward integration.** Rounding leaves the last sample before the end within single-precision residue of $q_f$; the hold beyond the end returns $q_f$ exactly.
- **Starts near the stopping point** may flip direction compared to the trapezoid: jerk lengthens the stopping distance, so a goal the trapezoid just reaches is overshot here.

## Variants & Generalizations

- **Rest start** ($v_0 = a_0 = 0$, $a_{max} = d_{max}$) reproduces the rest-to-rest [S-Curve Profile](SCurveProfile.md).
- **$j_{max} \to \infty$** collapses every jerk segment and approaches the [Trapezoidal Profile from a Moving State](TrapezoidalProfileFromState.md), except for the zero-crossing limit above.
- **Closed-form peak**: for $a_0 = 0$ and no zero crossing, $f(v_p) = \Delta'$ reduces to a quartic in $\sqrt{v_p}$ per plateau case; bisection is preferred for robustness.
- **Non-zero final velocity** follows by stopping phase 3 at the target velocity.

## Applications

- Position servos that accept a new setpoint mid-move (CiA 402 profile position mode with "change set immediately") while keeping the torque reference continuous.
- Retargeting a jerk-limited move from a supervisor or an operator input.

## Connections to Other Algorithms

- [S-Curve Profile](SCurveProfile.md): the rest-to-rest closed form.
- [Trapezoidal Profile from a Moving State](TrapezoidalProfileFromState.md): the same direction choice and replanning use, without the jerk limit.
- Bisection: the fixed-count bracketing root search that finds the peak velocity.

## References & Further Reading

- L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots*, Springer, 2008, §3.4 (double-S trajectory with initial and final velocities).
- T. Kröger, *On-Line Trajectory Generation in Robotic Systems*, Springer, 2010 (Reflexxes: online planning from an arbitrary state).

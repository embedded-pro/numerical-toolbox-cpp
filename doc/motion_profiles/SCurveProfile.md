# S-Curve (Jerk-Limited) Profile

## Overview & Motivation

A trapezoidal velocity profile bounds speed and acceleration, but acceleration jumps instantaneously at every corner. Those jumps are impulses in jerk: they excite structural resonances, wear gearing, and make payloads wobble or liquids spill. The S-curve (double-S) profile also bounds **jerk**, so acceleration ramps linearly in and out and is continuous, giving far less vibration than a trapezoid. The price is a slightly longer move.

For a rest-to-rest move the phase durations have a closed form: planning is a handful of arithmetic operations, with no iteration. Three limits drive it: a velocity ceiling $v_{\max}$, an acceleration ceiling $a_{\max}$ and a jerk ceiling $j_{\max}$. Short moves never reach the ceilings; the corresponding phases shrink to zero length.

## Mathematical Theory

### Phase Structure

A rest-to-rest move of length $h > 0$ is built from seven constant-jerk phases:

| Phase | Jerk | Acceleration | Duration |
|-------|------|--------------|----------|
| 1 | $+j_{\max}$ | ramps $0 \to a_{lim}$ | $T_j$ |
| 2 | $0$ | constant $a_{lim}$ | $T_a - 2T_j$ |
| 3 | $-j_{\max}$ | ramps $a_{lim} \to 0$ | $T_j$ |
| 4 | $0$ | zero, cruise at $v_{lim}$ | $T_v$ |
| 5 | $-j_{\max}$ | ramps $0 \to -a_{lim}$ | $T_j$ |
| 6 | $0$ | constant $-a_{lim}$ | $T_a - 2T_j$ |
| 7 | $+j_{\max}$ | ramps $-a_{lim} \to 0$ | $T_j$ |

Phases 1–3 form the acceleration stage of length $T_a$; phases 5–7 mirror it, so the profile is symmetric about its midpoint and the total time is $T = 2T_a + T_v$. Integrating the stage gives the peaks and the distance:

$$a_{lim} = j_{\max} T_j, \qquad v_{lim} = a_{lim}\,(T_a - T_j), \qquad h = v_{lim}\,(T_a + T_v)$$

### Closed-Form Three-Step Planner

This is the case $v_0 = v_1 = 0$ of Biagiotti and Melchiorri, §3.4. Each step assumes one more ceiling is unreachable.

**Step 1 — assume $v_{\max}$ is reached** ($v_{lim} = v_{\max}$):

$$
\text{if } v_{\max} j_{\max} \ge a_{\max}^2:\; T_j = \frac{a_{\max}}{j_{\max}},\; T_a = T_j + \frac{v_{\max}}{a_{\max}}
\qquad\text{else}\qquad
T_j = \sqrt{\frac{v_{\max}}{j_{\max}}},\; T_a = 2T_j
$$

$$T_v = \frac{h}{v_{\max}} - T_a$$

The first case has an acceleration plateau; the second has a pure triangular ramp with peak $j_{\max}T_j < a_{\max}$. Here $T = h/v_{\max} + T_a$, so the jerk limit costs exactly $a_{\max}/j_{\max}$ over a trapezoid with the same $v_{\max}, a_{\max}$.

**Step 2 — if $T_v < 0$**, set $T_v = 0$ and assume $a_{\max}$ is still reached, $T_j = a_{\max}/j_{\max}$. Then $h = a_{\max} T_a (T_a - T_j)$ is a quadratic in $T_a$:

$$\Delta = \frac{a_{\max}^4}{j_{\max}^2} + 4\,a_{\max} h, \qquad T_a = \frac{a_{\max}^2/j_{\max} + \sqrt{\Delta}}{2\,a_{\max}}$$

**Step 3 — if $T_a < 2T_j$**, the plateau would have negative length, so $a_{\max}$ is unreachable too. Both ramps are triangles, $T_a = 2T_j$ and $h = 2 j_{\max} T_j^3$:

$$T_j = \sqrt[3]{\frac{h}{2\,j_{\max}}}, \qquad T_a = 2T_j$$

Step 2 applies when $h < v_{\max} T_a$, and step 3 when $h < 2 a_{\max}^3 / j_{\max}^2$ (from $T_a \ge 2T_j \iff \Delta \ge 9 a_{\max}^4/j_{\max}^2$). Neighbouring branches agree on $T$ at each threshold, so $T$ is continuous in $h$. No duration is ever negative: in step 1, $T_a - 2T_j = v_{\max}/a_{\max} - a_{\max}/j_{\max} \ge 0$ is the branch condition itself; step 2 is guarded by the step-3 test; step 3 has $T_a - 2T_j = 0$. Zero-length phases collapse out of the profile.

### Exact Evaluation

Within a phase of jerk $j_i$ starting from state $(s_i, v_i, a_i)$, with local time $\tau$:

$$a = a_i + j_i\tau, \qquad v = v_i + a_i\tau + \tfrac12 j_i\tau^2, \qquad s = s_i + v_i\tau + \tfrac12 a_i\tau^2 + \tfrac16 j_i\tau^3$$

The seven boundary states are integrated exactly once, at planning time, with these polynomials. Each sample then locates its phase and evaluates a cubic in time. At and beyond $T$ the profile holds the goal at rest; before $t = 0$ it holds the start. A negative move direction negates the offset, velocity, acceleration and jerk.

### Time Scaling for Synchronization

Replanning with $(v_{\max}/\lambda,\; a_{\max}/\lambda^2,\; j_{\max}/\lambda^3)$ stretches an axis by $\lambda \ge 1$. Every planner term scales consistently ($T_j \to \lambda T_j$, $T_a \to \lambda T_a$, $\Delta \to \lambda^{-2}\Delta$) and the branch tests are invariant, so the duration scales by exactly $\lambda$ in every branch. Axes stretched to a common $T$ finish together.

## Complexity Analysis

| Operation | Time | Space | Notes |
|-----------|------|-------|-------|
| Plan | $O(1)$ | $O(1)$ | At most one square root and one cube root, no iteration |
| Sample | $O(1)$ | $O(1)$ | Locate one of seven phases, evaluate one cubic |

Stored state is a few fixed-size tables of phase boundaries on the stack; no heap is used. Cost is independent of $h$ and of the branch taken.

## Step-by-Step Walkthrough

Limits $(v_{\max}, a_{\max}, j_{\max}) = (1, 2, 10)$, $h = 0.3$.

1. Step 1: $v_{\max} j_{\max} = 10 \ge 4$, so $T_j = 0.2$, $T_a = 0.7$, $T_v = 0.3 - 0.7 = -0.4 < 0$.
2. Step 2: $T_v = 0$, $\Delta = 0.16 + 2.4 = 2.56$, $T_a = (0.4 + 1.6)/4 = 0.5 \ge 2T_j = 0.4$, so step 3 is not needed. Then $a_{lim} = 2$, $v_{lim} = 2(0.5 - 0.2) = 0.6$, $T = 1.0$.

Boundary states $(a, v, s)$ of the acceleration stage: at $t = 0.2$, $(2,\, 0.2,\, 0.0133)$; at $t = 0.3$, $(2,\, 0.4,\, 0.0433)$; at $t = 0.5$, $(0,\, 0.6,\, 0.15)$. The mirrored stage ends at $s = 0.3$, at rest. A sample at $t = 0.4$ lies in phase 3 with $\tau = 0.1$: $a = 1$, $v = 0.55$, $s = 0.0917$.

| $(v_{\max}, a_{\max}, j_{\max})$ | $h$ | Branch | $T_j$ | $T_a$ | $T_v$ | $T$ |
|----------------------------------|-----|--------|-------|-------|-------|-----|
| (1, 2, 10) | 5 | 1, plateau | 0.2 | 0.7 | 4.3 | 5.7 |
| (1, 2, 1) | 5 | 1, triangle | 1 | 2 | 3 | 7 |
| (1, 2, 10) | 0.3 | 2 | 0.2 | 0.5 | 0 | 1.0 |
| (1, 2, 10) | 0.02 | 3 | 0.1 | 0.2 | 0 | 0.4 |

For $(1, 2, 10)$ the thresholds are $h \ge 0.7$ for step 1 and $h \ge 2a_{\max}^3/j_{\max}^2 = 0.16$ for step 2. The last row has $a_{lim} = 1$ and $v_{lim} = 0.1$.

## Pitfalls & Edge Cases

- **Positive limits.** The velocity, acceleration and jerk ceilings must all be strictly positive.
- **Zero distance.** With $h = 0$ every phase has zero length; the profile is the start state, at rest.
- **Rest-to-rest only.** Non-zero boundary velocities need the iterative acceleration reduction of the general case, which is not covered.
- **Float-only.** Only floating-point types are supported; there is no fixed-point variant.
- **Validation.** On 20 000 random $(h, v_{\max}, a_{\max}, j_{\max})$ plans, every plan ended at $h$ at rest and never exceeded the limits.

## Variants & Generalizations

- **Trapezoid** — the limit $j_{\max} \to \infty$: jerk phases vanish and acceleration steps.
- **Non-zero boundary velocities** — same seven phases, with the acceleration ceiling reduced iteratively until the move is feasible.

## Applications

- **Pick-and-place, CNC, semiconductor handling** — fast moves without exciting resonances or payload sway; time-scaled axes finish together.

## Connections to Other Algorithms

| Algorithm | Relationship |
|-----------|--------------|
| Trapezoidal profile | Jerk-unbounded predecessor; shares the synchronization scheme |
| Quintic polynomial profile | Fixed duration and smooth, but limits are not enforced directly |

## References & Further Reading

- Biagiotti, L. and Melchiorri, C., *Trajectory Planning for Automatic Machines and Robots*, Springer, 2008, §3.4.

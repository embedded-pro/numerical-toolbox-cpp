# Angle Utilities

## Overview & Motivation

Angles live on a circle, not a line: a heading of $179^\circ$ and one of $-179^\circ$ are $2^\circ$ apart, not $358^\circ$. Controllers that subtract angles naively command a full turn the wrong way, integrators that never wrap lose precision as the value grows, and encoders or IMUs report wrapped values that an estimator must turn back into a continuous signal. These bugs recur in every module that touches angles, so the library provides one stateless, tested set of helpers with one convention: wrap into a canonical interval, take the shortest signed difference, and unwrap a wrapped measurement.

## Mathematical Theory

### Wrapping

Two angles are equivalent when they differ by a multiple of $2\pi$. Wrapping picks the representative inside a fixed half-open interval of length $2\pi$:

$$\operatorname{wrap}_{(-\pi,\pi]}(a) = r - \pi,\;\; r \in (0, 2\pi],\; r \equiv a + \pi \pmod{2\pi} \qquad \operatorname{wrap}_{[0,2\pi)}(a) = a \bmod 2\pi$$

The floating-point remainder (fmod) is *truncated* — it keeps the sign of the dividend. The implementation therefore shifts the angle so the interval starts at zero, takes the remainder $r \in (-2\pi, 2\pi)$, adds $2\pi$ if $r$ is not positive (signed case) or is negative (unsigned case), and shifts back.

**Convention (normative):** the signed interval is $(-\pi, \pi]$; both $+\pi$ and $-\pi$ map to $+\pi$, so a half turn has exactly one representation. The unsigned interval is $[0, 2\pi)$ and never returns $2\pi$.

### Shortest signed difference

For target $\theta_t$ and current $\theta_c$:

$$\Delta(\theta_t, \theta_c) = \operatorname{wrap}_{(-\pi,\pi]}(\theta_t - \theta_c)$$

Its magnitude is the shortest arc and its sign is the direction of travel from $\theta_c$ to $\theta_t$.

### Unwrapping

Given the previous continuous value $u_{k-1}$ and a new wrapped sample $w_k$:

$$u_k = u_{k-1} + \Delta(w_k, u_{k-1})$$

This recovers the true continuous angle as long as consecutive true samples differ by less than $\pi$; otherwise the step is aliased by a multiple of $2\pi$.

## Complexity Analysis

| Operation            | Time   | Space  | Notes                                          |
|----------------------|--------|--------|------------------------------------------------|
| Wrap to $(-\pi,\pi]$ | $O(1)$ | $O(1)$ | One fmod, one compare, a few adds              |
| Wrap to $[0,2\pi)$   | $O(1)$ | $O(1)$ | One fmod, two compares, one conditional add    |
| Shortest difference  | $O(1)$ | $O(1)$ | One subtraction plus one signed wrap           |
| Unwrap               | $O(1)$ | $O(1)$ | One shortest difference plus one add; no state |

Cost does not depend on the magnitude of the input: the remainder is a single operation, not a loop subtracting $2\pi$.

## Step-by-Step Walkthrough

**Shortest difference, target $170^\circ$, current $-170^\circ$:** plain difference $= 340^\circ$. Shift: $340 + 180 = 520$. Remainder mod $360$: $160$ (positive, no correction). Shift back: $160 - 180 = -20^\circ$. The short way is $20^\circ$ in the negative direction, not $340^\circ$ forward.

**Wrap, $7\pi/3$:** shift: $7\pi/3 + \pi = 10\pi/3$. Remainder mod $2\pi$: $10\pi/3 - 2\pi = 4\pi/3$. Shift back: $4\pi/3 - \pi = \pi/3$. The unsigned wrap gives the same $\pi/3$.

**Half turn:** $a = -\pi$ shifts to $0$; the remainder is $0$, which is not positive, so $2\pi$ is added and the result is $2\pi - \pi = +\pi$.

**Unwrap, previous $358^\circ$, new sample $2^\circ$:** $\Delta = \operatorname{wrap}(2 - 358) = \operatorname{wrap}(-356) = +4^\circ$, so the continuous value is $362^\circ$.

## Pitfalls & Edge Cases

**Precision degrades with $|a|$** — the remainder itself is exact, but $2\pi$ is not representable, so each full turn removed carries the representation error of $2\pi$ (about $1.7\times10^{-7}$ rad in single precision) and the total error grows like $|a|\,\varepsilon$. Wrap at the source, every integration step, rather than after long accumulation. An unwrapped signal is unbounded by design, so its own resolution shrinks as it grows.

**Aliasing in unwrap** — if the true angle moves by $\pi$ or more between samples, the direction is ambiguous and the result is off by a multiple of $2\pi$. Sample faster than half a turn per period.

**Half-turn asymmetry** — because $\pm\pi$ both map to $+\pi$, the shortest difference is not antisymmetric at exactly a half turn: swapping the arguments gives $+\pi$ again, not $-\pi$.

**Rounding at the unsigned boundary** — a tiny negative remainder plus $2\pi$ can round to exactly $2\pi$; the unsigned wrap detects this and returns $0$, so the result stays strictly inside $[0, 2\pi)$.

**Non-finite input** — infinities and NaN have no meaningful remainder; inputs must be finite.

**Float-only** — floating-point types only, enforced at compile time; no fixed-point variants.

## Variants & Generalizations

Degrees or turns as the unit only rescale the period ($360$ or $1$ instead of $2\pi$); the unit-turn form avoids the inexact $2\pi$ entirely. Other canonical intervals, such as $[-\pi, \pi)$, differ only in which half-turn representative is kept. For rotations in 3D, quaternions avoid the wrap problem altogether, at the cost of a double cover ($q$ and $-q$).

## Applications

- **Heading and attitude control** — error terms computed as the shortest difference never command the long way round.
- **Sensor fusion** — the complementary filter's heading mode blends the measurement along the shortest arc and wraps the estimate back into $(-\pi, \pi]$.
- **Encoder and IMU processing** — unwrap a wrapped reading to track multi-turn position.
- **Odometry and inverse kinematics** — canonical joint angles with a single representation of a half turn.

## Connections to Other Algorithms

Used by the complementary filter in its heading mode. Related to the quaternion rotation representation, which has no wrap discontinuity.

## References & Further Reading

- Goldberg, D., "What every computer scientist should know about floating-point arithmetic," ACM Computing Surveys 23(1), 1991 (range reduction and the cost of an inexact $2\pi$)

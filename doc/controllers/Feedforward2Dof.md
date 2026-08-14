# Feedforward / 2-DOF Controller

## Overview & Motivation

A single feedback loop must balance two competing demands: fast reference tracking and robust rejection of disturbances. Tightening the feedback gains for faster tracking reduces stability margins; relaxing them for robustness slows the response. The two-degree-of-freedom (2-DOF) structure resolves this conflict by separating the two tasks into independent paths. A feedforward path shapes how the output follows the reference, while the feedback path corrects errors and rejects disturbances. Because the feedforward term is open-loop, it can be tuned for tracking without affecting the closed-loop stability margins at all.

## Mathematical Theory

### Signal Flow

Let $r$ be the reference, $y$ the plant output, and $u$ the actuator command. The 2-DOF law is:

$$u = \text{sat}\!\left(u_{ff}(r) + u_{fb}(r - y),\; u_{min},\; u_{max}\right)$$

where:
- $u_{ff}(r)$ is the feedforward map evaluated on the reference alone (open-loop),
- $u_{fb}(e) = u_{fb}(r - y)$ is the feedback law driven by the tracking error,
- $\text{sat}(\cdot)$ clips the combined command to the actuator range $[u_{min}, u_{max}]$.

### Feedforward Design

The ideal feedforward is an inverse of the plant model. For a linear plant $P(z)$, the ideal map is $u_{ff}(r) = P^{-1}(z)\, r$, yielding perfect steady-state tracking. A simpler approximation is a scalar gain $K_{ff}$ tuned to match the DC plant gain: $u_{ff}(r) = K_{ff} \cdot r$. Non-minimum-phase plants require a causal approximation since the exact inverse is non-causal.

### Feedback Design (Independence)

The closed-loop characteristic equation depends only on the feedback law and the plant; $u_{ff}$ does not appear in it. Stability margins, bandwidth, and disturbance rejection are therefore set entirely by $u_{fb}$. This decoupling is the defining property of the 2-DOF structure.

### Saturation

The output clamp $\text{sat}(u, u_{min}, u_{max})$ acts on the combined command to protect the actuator. When the sum saturates, both the feedforward and feedback contributions are implicitly reduced.

## Complexity Analysis

| Case    | Time   | Space  | Notes                                        |
|---------|--------|--------|----------------------------------------------|
| Best    | $O(1)$ | $O(1)$ | Wrapper adds one addition and one clamp      |
| Average | $O(1)$ | $O(1)$ | Cost dominated by the injected components    |
| Worst   | $O(1)$ | $O(1)$ | State lives entirely inside injected objects |

## Step-by-Step Walkthrough

Given $r = 0.5$, $y = 0.1$, feedforward gain $K_{ff} = 0.6$, and a proportional feedback with gain $K_p = 0.5$, clamped to $[-1, 1]$:

1. $u_{ff} = K_{ff} \cdot r = 0.6 \times 0.5 = 0.3$
2. $e = r - y = 0.5 - 0.1 = 0.4$
3. $u_{fb} = K_p \cdot e = 0.5 \times 0.4 = 0.2$
4. $u_{raw} = u_{ff} + u_{fb} = 0.3 + 0.2 = 0.5$
5. $u = \text{sat}(0.5, -1, 1) = 0.5$

If the system were already tracking perfectly ($y = r = 0.5$), then $e = 0$, $u_{fb} = 0$, and the feedforward alone supplies the command: $u = K_{ff} \cdot 0.5 = 0.3$.

## Pitfalls & Edge Cases

- **Saturation interacts with both terms.** When the actuator saturates, the feedback integrator (if used) can wind up. Anti-windup logic in the feedback law is recommended.
- **Feedforward does not aid disturbance rejection.** If $u_{ff}$ is tuned for tracking and a disturbance enters the plant, only $u_{fb}$ responds.
- **Inverse-plant feedforward for non-minimum-phase systems** is non-causal; use a truncated or approximate inverse.
- **Sign convention.** The error passed to feedback is $r - y$, not $y - r$. Swapping signs destabilises positive-gain plants.
- **Stateful feedforward must implement `Reset()`.** Dynamic pre-filters (e.g. reference model filters) carry internal state that must be cleared on `Reset()`. Both the feedforward and feedback components are reset when `Feedforward2Dof::Reset()` is called; any feedforward implementation that holds state must override `Reset()` to clear it.

## Variants & Generalizations

- **Gain-only feedforward:** $u_{ff} = K_{ff} \cdot r$ — the simplest form, requiring only a scalar.
- **Filter-based feedforward:** a low-pass or derivative filter applied to $r$ before multiplication; smooths the command for actuators with bandwidth limits.
- **Inverse-plant feedforward:** uses a model of the plant to produce the exact command required; maximises tracking performance at the cost of model accuracy sensitivity.
- **Model-reference 2-DOF:** the feedforward block is an explicit reference model; the feedback corrects deviations from the model response rather than from the raw reference.

## Applications

- Motion control: hitting aggressive position or velocity trajectories while maintaining stability against load disturbances.
- Process control: tracking ramp set-points in temperature or flow loops where a simple P or PI controller lags behind.
- Robotics: feed-forward gravity/inertia compensation combined with joint-level PD feedback.
- Power electronics: duty-cycle feedforward in DC-DC converters to cancel input voltage disturbances before the voltage-mode feedback acts.

## Connections to Other Algorithms

- **PID / PidIncremental** is a natural choice for the injected feedback law. The incremental (velocity) form avoids integrator wind-up issues at start-up.
- **SaturationRateLimiter** can replace or augment the output clamp to add slew-rate limiting on the command.
- **Gain-scheduled controller** extends the 2-DOF concept by varying feedforward and feedback gains across operating points.
- **LQR / LQG** provides an optimal state-feedback law for the feedback path; the feedforward term then handles reference shaping independently.

## References & Further Reading

- K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2008, Ch. 12 (Feedforward and 2-DOF Design).
- G. F. Franklin, J. D. Powell, and A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019, Ch. 4.

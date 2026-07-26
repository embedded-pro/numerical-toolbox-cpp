# Saturation / Rate-Limiter / Slew Blocks

## Overview & Motivation

Real actuators — valves, motors, heaters, servos — impose two distinct constraints on any command signal: a travel limit (the actuator cannot physically move past a hard stop) and a speed limit (the mechanism cannot slew arbitrarily fast without damaging itself or the system). Ignoring either constraint causes physically impossible commands, mechanical stress, and, in closed-loop systems, integrator wind-up that degrades recovery after saturation.

These three primitives model exactly those constraints. Saturation enforces the travel limit by clipping the output to a fixed interval. The rate limiter enforces the speed limit by allowing the output to move at most a fixed amount per sample period. Composing them — applying the rate limiter first, then the saturator — guarantees every emitted sample simultaneously respects both bounds, making them the mandatory prerequisite for anti-windup in any integrating controller.

## Mathematical Theory

### Saturation

Given bounds $[\ell, h]$ with $\ell < h$, saturation is defined as:

$$y = \text{sat}(u) = \min\!\bigl(\max(u,\, \ell),\, h\bigr)$$

This is a memoryless, instantaneous projection onto the interval $[\ell, h]$.

### Rate Limiter (Slew Limiter)

Let $\Delta_{\max} = r \cdot T_s$ be the maximum permitted change per sample, where $r$ is the slew rate (units per second) and $T_s$ is the sample period (seconds). Define the state $y[n-1]$ as the last emitted output. The output at tick $n$ is:

$$\delta[n] = \text{sat}_{\Delta_{\max}}(u[n] - y[n-1]) = \min\!\bigl(\max(u[n] - y[n-1],\,-\Delta_{\max}),\,+\Delta_{\max}\bigr)$$

$$y[n] = y[n-1] + \delta[n]$$

On the first sample (priming), $y[0] = u[0]$ and no limiting is applied, so the output tracks the first command exactly.

### Composed Block (Slew-Then-Clamp)

$$y_{\text{composed}}[n] = \text{sat}\!\bigl(y_{\text{slew}}[n]\bigr)$$

Applying the rate limiter before the saturator is essential. If the order were reversed (clamp then slew), a step command to a saturated bound would allow the rate limiter to move the output outside the saturation interval for one tick before the clamp could correct it.

## Complexity Analysis

| Case    | Time   | Space  | Notes                                         |
|---------|--------|--------|-----------------------------------------------|
| Best    | $O(1)$ | $O(1)$ | Two compare-and-select operations             |
| Average | $O(1)$ | $O(1)$ | One multiply + two compares for rate limit    |
| Worst   | $O(1)$ | $O(1)$ | Fixed instruction count; no branching on size |

Each block maintains at most two scalar parameters and one scalar state word. The hot path is branch-light: `min`/`max` compile to conditional-move instructions on modern architectures.

## Step-by-Step Walkthrough

Parameters: $\ell = -0.5$, $h = 0.5$, $r = 0.1\,\text{s}^{-1}$, $T_s = 1\,\text{s}$ ($\Delta_{\max} = 0.1$). Step request: $u = 1.0$ for all ticks, starting from $y[-1] = 0$.

| Tick $n$ | $u[n]$ | $\delta[n]$ | $y_{\text{slew}}[n]$ | $y_{\text{composed}}[n]$ |
|----------|--------|-------------|----------------------|--------------------------|
| 0        | 1.0    | 0.1         | 0.1                  | 0.1                      |
| 1        | 1.0    | 0.1         | 0.2                  | 0.2                      |
| 2        | 1.0    | 0.1         | 0.3                  | 0.3                      |
| 3        | 1.0    | 0.1         | 0.4                  | 0.4                      |
| 4        | 1.0    | 0.1         | 0.5                  | 0.5                      |
| 5        | 1.0    | 0.1         | 0.6                  | 0.5 (clamped)            |

The ramp is linear until the saturator clips it at $h = 0.5$. The rate limiter continues to advance its internal state, but the composed output is held at the bound.

## Pitfalls & Edge Cases

- **Zero slew rate**: $r = 0$ means $\Delta_{\max} = 0$; the output freezes at the primed value for all subsequent inputs. This is valid for a hold block but surprising if unintentional.
- **Collapsed saturation** ($\ell = h$): every output equals that constant regardless of input. The rate limiter still advances but the saturator immediately overwrites the result.
- **Priming semantics**: the first call to the rate limiter always passes through without limiting. Systems that require immediate rate-limiting from the first sample must call `Reset` with the desired initial state before the first control tick.
- **Order of composition**: slew-then-clamp is the only order that guarantees simultaneous satisfaction of both constraints at every tick.
- **Floating-point accumulation**: repeated addition of $\delta[n]$ can accumulate rounding error over many ticks. For long-running loops at high rate, the error remains bounded by machine epsilon times the number of steps times $\Delta_{\max}$.

## Variants & Generalizations

- **Asymmetric rate limiter**: use separate positive and negative slew rates $r^+$, $r^-$ to model actuators with different rise and fall speeds (e.g., a valve that opens slowly but closes fast).
- **Variable-rate limiter**: replace the fixed $\Delta_{\max}$ with a schedule or feedback signal for gain-scheduled slew limiting.
- **Deadband + saturation**: insert a deadband nonlinearity before the saturator to model mechanical backlash or sensor noise floors.

## Applications

- **Motor drives**: limit acceleration commands to prevent over-current trips while hard-bounding the velocity or position command within safe travel.
- **Thermal control**: ramp heater power slowly to avoid thermal shock while clamping to the maximum safe power level.
- **Anti-windup**: feed the saturated (clamped) output back to the integrator of a PID controller so accumulation stops when the actuator is pinned at a bound.
- **Sensor fusion pre-processing**: slew-limit noisy sensor inputs before they enter an estimator to attenuate impulsive outliers without a dedicated filter.

## Connections to Other Algorithms

- **PidIncremental**: consumes the clamped output for anti-windup; the saturation block provides the feedback signal that stops integrator accumulation.
- **BangBangHysteresis**: an alternative, discontinuous actuator constraint that switches between two fixed outputs; saturation is the continuous analog.
- **ExponentialMovingAverage**: can approximate a low-pass slew effect but does not enforce a hard rate bound and has no priming concept.

## References & Further Reading

- K. J. Astrom, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers* (2008), sections on actuator saturation and integrator windup.
- K. J. Astrom, T. Hagglund, *Advanced PID Control* (2006), Chapter 6: Anti-windup.

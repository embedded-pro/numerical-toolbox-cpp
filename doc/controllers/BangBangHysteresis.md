# Bang-Bang / Hysteresis (Relay) Controller

## Overview & Motivation

The bang-bang controller is the simplest closed-loop regulator: its output switches between two discrete levels depending on whether the controlled variable is above or below a threshold. Without hysteresis, a plain threshold comparator chatters — switching at high frequency whenever noise nudges the signal across the boundary.

Adding a **dead-band (Schmitt trigger)** eliminates chatter by giving the relay memory: once the output goes High it stays High until the measurement falls all the way to a *lower* threshold, and vice versa. This makes the relay a practical actuator-safe control primitive for thermostats, fridge compressors, tank level switches, and power-stage on/off regulation.

## Mathematical Theory

### State Transition

The relay holds a binary state $s \in \{\text{Low}, \text{High}\}$ with the following transition rules:

$$
s[k] = \begin{cases}
\text{High} & \text{if } s[k-1] = \text{Low} \text{ and } x[k] \geq \theta_H \\
\text{Low}  & \text{if } s[k-1] = \text{High} \text{ and } x[k] \leq \theta_L \\
s[k-1]      & \text{otherwise}
\end{cases}
$$

where $\theta_L < \theta_H$ are the lower and upper switching thresholds (the hysteresis band).

### Output Map

$$
u[k] = \begin{cases}
u_H & \text{if } s[k] = \text{High} \\
u_L & \text{if } s[k] = \text{Low}
\end{cases}
$$

The output levels $u_L$ and $u_H$ are arbitrary; common choices are $\{0, 1\}$ or $\{-1, +1\}$.

### Hysteresis Band Width

The band width $\Delta = \theta_H - \theta_L$ is the key design parameter. It bounds the switching frequency $f_s$ given a signal slope $\dot{x}$:

$$
f_s \leq \frac{|\dot{x}|}{2\Delta}
$$

A wider band reduces $f_s$ (protecting relays and power stages) at the cost of a larger steady-state limit cycle amplitude.

## Complexity Analysis

| Case | Time   | Space  | Notes                                      |
|------|--------|--------|--------------------------------------------|
| All  | $O(1)$ | $O(1)$ | Two comparisons, one state bit, one select |

No arithmetic on the signal path — only comparisons — so the relay introduces no numerical error and is exactly representable in any floating-point format.

## Step-by-Step Walkthrough

**Setup:** band $[\theta_L, \theta_H] = [-0.2, 0.2]$, outputs $u_L = 0$, $u_H = 1$, initial state Low.

| Step | $x[k]$ | Condition                    | $s[k]$ | $u[k]$ |
|------|--------|------------------------------|--------|--------|
| 1    | 0.0    | Low, $x < 0.2$               | Low    | 0      |
| 2    | 0.3    | Low, $x \geq 0.2$ → switch   | High   | 1      |
| 3    | 0.1    | High, $x > -0.2$ → hold      | High   | 1      |
| 4    | −0.3   | High, $x \leq -0.2$ → switch | Low    | 0      |
| 5    | 0.0    | Low, $x < 0.2$ → hold        | Low    | 0      |

## Pitfalls & Edge Cases

- **Inverted band.** $\theta_H \leq \theta_L$ latches the output in an undefined state; reject this at construction time via a precondition assertion.
- **Exactly on threshold.** Transitions are inclusive: $x = \theta_H$ triggers Low→High and $x = \theta_L$ triggers High→Low. This avoids a dead-zone at the switching points.
- **Zero-width band.** $\theta_H = \theta_L$ collapses the relay to a pure comparator (no hysteresis). The logic is still correct but offers no chatter suppression.
- **Noise sizing.** The band width must exceed the peak-to-peak noise amplitude; otherwise noise alone drives state transitions at the sampling rate.
- **Actuator minimum on-time.** Size $\Delta$ so that the minimum on-time (derived from $\Delta / |\dot{x}|_\text{max}$) is above the actuator's rated minimum switching period.

## Variants & Generalizations

| Variant                                   | Key Difference                                                                                            |
|-------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| **Plain comparator**                      | $\Delta = 0$; no memory, chatters on noise                                                                |
| **Asymmetric band**                       | $\theta_H$ and $\theta_L$ not symmetric around the set-point; biases the duty cycle                       |
| **Three-state relay**                     | Adds a dead-band output level $u_0$; used in motor direction control                                      |
| **Adaptive hysteresis**                   | Band width tracks signal variance online to maintain a target switching rate                              |
| **Relay feedback test (Åström–Hägglund)** | Deliberate oscillation under relay feedback to identify the ultimate gain/period for automatic PID tuning |

## Applications

- **Thermostats and HVAC** — heating/cooling switched on/off around a temperature set-point.
- **Tank and vessel level control** — pump on/off between high- and low-level floats.
- **Power-stage converters** — hysteretic current-mode control in DC-DC converters and class-D amplifiers.
- **Motor drive enable/disable** — protecting power stages with a current-band relay.
- **Åström–Hägglund auto-tuning** — the relay feedback experiment that drives limit-cycle oscillation for PID parameter identification.

## Connections to Other Algorithms

| Algorithm                                             | Relationship                                                                                                      |
|-------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------|
| [PID Controller](Pid.md)                              | The relay's limit cycle can be used to identify PID tuning parameters via the Åström–Hägglund relay-feedback test |
| [Saturation / Rate Limiter](SaturationRateLimiter.md) | Continuous-output counterpart for actuator constraint; often combined with a relay in cascaded loops              |

## References & Further Reading

- K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2008 — relay feedback, Chapter 10.
- Ya. Z. Tsypkin, *Relay Control Systems*, Cambridge University Press, 1984.
- K. J. Åström, T. Hägglund, "Automatic Tuning of Simple Regulators with Specifications on Phase and Amplitude Margins," *Automatica*, 20(5), 1984.

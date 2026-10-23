# Friction Compensation (Coulomb + Viscous + Stribeck)

## Overview & Motivation

Friction is the dominant unmodelled effect in geared drives, lead screws and robot joints. It causes stick-slip, tracking lag and steady-state error that a feedback loop can only remove by raising its gain, which erodes stability margins. Friction compensation instead predicts the drag from the axis velocity and adds the opposite torque (or force) to the command, so the feedback loop only has to correct the residual.

The model superposes three effects: **Coulomb** drag (constant magnitude, opposing motion), **viscous** drag (proportional to speed) and **Stribeck** stiction (an extra breakaway level that fades as the axis starts to slide). The ideal curve is discontinuous at zero velocity, so a smooth boundary layer replaces the hard sign function.

## Mathematical Theory

### Per-Axis Model

For one axis with velocity $v$, the compensation torque is:

$$\tau = \operatorname{clamp}\!\Big(\big[F_c + (F_s - F_c)\,e^{-(|v|/v_s)^{\delta}}\big]\tanh\!\big(v/\varepsilon\big) + F_v\, v,\; -\tau_{max},\; \tau_{max}\Big)$$

- $F_c$: Coulomb level, the drag magnitude once sliding.
- $F_s \ge F_c$: stiction (breakaway) level at near-zero speed.
- $F_v$: viscous coefficient.
- $v_s > 0$: Stribeck velocity, the speed scale over which stiction decays into the Coulomb plateau.
- $\delta$: Stribeck shape exponent, typically 1 (exponential decay) or 2 (Gaussian decay).
- $\varepsilon > 0$: boundary-layer width replacing $\operatorname{sign}(v)$.
- $\tau_{max} > 0$: output cap.

Axes are independent; a multi-axis compensator applies the model to each velocity component with its own parameters.

### Structure

The bracketed term is the friction **level** $L(|v|)$. It starts at $F_s$ for $v = 0$ and decays monotonically to $F_c$, which is why $F_s \ge F_c$ is required: otherwise the "Stribeck dip" would turn into a rise. The factor $\tanh(v/\varepsilon)$ supplies the direction, approximating $\operatorname{sign}(v)$ with slope $L/\varepsilon$ at the origin. The viscous term is added outside the boundary layer.

### Feedforward Use

The result is a feedforward term, not a feedback law. It is added to the controller output, $\tau_{cmd} = \tau_{control} + \tau$, and composes with any torque or current loop. Because it depends only on velocity, it does not enter the feedback characteristic equation.

### Desired vs. Measured Velocity

- **Desired velocity** gives a noise-free, deterministic command, but cannot see actual sticking, so it mis-compensates when the axis lags the trajectory.
- **Measured velocity** tracks the real motion, but passes sensor noise through the boundary-layer slope $\approx F_s/\varepsilon$, and closes an extra velocity-feedback path whose delay can excite limit cycles.

In both cases $\varepsilon$ sets the sharpness-versus-chatter trade-off.

### Identification

For fixed $v_s$, $\delta$ and $\varepsilon$, $\tau$ is linear in the force coefficients: $F_c$, $F_s$ and $F_v$ multiply the known regressors $(1-e^{-(|v|/v_s)^{\delta}})\tanh(v/\varepsilon)$, $e^{-(|v|/v_s)^{\delta}}\tanh(v/\varepsilon)$ and $v$ respectively. They can therefore be estimated by linear least squares from logged velocity and torque data, jointly with other linear parameters. $v_s$ and $\delta$ require an outer search.

## Complexity Analysis

| Case | Time   | Space  | Notes                                                                                                                                      |
|------|--------|--------|--------------------------------------------------------------------------------------------------------------------------------------------|
| All  | $O(N)$ | $O(N)$ | $N$ axes; per axis one exponential, one power, one hyperbolic tangent, two divisions and a clamp; $N$ parameter sets stored, no allocation |

Cost is dominated by the three transcendental calls; there is no iteration and no dependence on history, so the latency is constant per sample.

## Step-by-Step Walkthrough

Parameters: $F_c = 0.5$, $F_s = 0.8$, $F_v = 0.1$, $v_s = 0.05$, $\delta = 2$, $\varepsilon = 10^{-3}$, with a non-binding cap $\tau_{max} = 1$.

| $v$           | $(\lvert v\rvert/v_s)^2$ | Level $L$                            | $\tanh(v/\varepsilon)$ | $F_v v$ | $\tau$ |
|---------------|--------------------------|--------------------------------------|------------------------|---------|--------|
| 1             | 400                      | $0.5 + 0.3\,e^{-400} \approx 0.5$    | 1                      | 0.1     | 0.6    |
| 0.01          | 0.04                     | $0.5 + 0.3\,e^{-0.04} \approx 0.788$ | $\tanh 10 \approx 1$   | 0.001   | 0.789  |
| 0.05 ($=v_s$) | 1                        | $0.5 + 0.3/e \approx 0.610$          | $\tanh 50 \approx 1$   | 0.005   | 0.615  |

At high speed the Stribeck term has vanished and only Coulomb plus viscous drag remain. At $v = 0.01$ the axis is still deep in the Stribeck region, so the level is close to $F_s$. At $v = v_s$ the stiction excess has decayed to a fraction $1/e$ of its initial value. At $v = 0$ the output is exactly zero, and $v = -0.01$ gives $-0.789$ by symmetry.

## Pitfalls & Edge Cases

- **Boundary layer too narrow.** $\varepsilon \to 0$ recovers $\operatorname{sign}(v)$, so noise and direction reversals toggle the full $\pm L$ each sample, producing chattering and limit cycles around zero velocity.
- **Boundary layer too wide.** For $|v| \lesssim \varepsilon$ the factor $\tanh(v/\varepsilon)$ is well below 1 (0.76 at $|v| = \varepsilon$), so slow moves and the breakaway region are under-compensated.
- **Over-compensation is worse than under-compensation.** An inflated estimate injects energy and can drive the axis on its own, while a low one merely leaves residual error. The clamp at $\tau_{max}$ bounds this; it should still exceed the true drag at the maximum operating speed, or it truncates the viscous term.
- **Parameter validation.** $F_s \ge F_c$, $v_s > 0$, $\varepsilon > 0$ and $\tau_{max} > 0$ are checked at construction and a violation is a precondition failure. The shape exponent is not checked; it must be positive, since $\delta \le 0$ removes the decay or diverges at $v = 0$.
- **Symmetry.** The level depends on $|v|$ only, and $\tanh$, the viscous term and the symmetric clamp are all odd, so $\tau(-v) = -\tau(v)$ and $\tau(0) = 0$ exactly. Real drives often have direction-dependent friction, which this model cannot represent.
- **Float precision.** For large $|v|/v_s$ the exponential underflows to zero harmlessly, and $\tanh$ saturates to $\pm 1$; no cancellation arises.
- **Clamp breaks linearity.** Identification data must be taken where the clamp is inactive.

## Variants & Generalizations

| Variant                    | Key Difference                                                                                                       |
|----------------------------|----------------------------------------------------------------------------------------------------------------------|
| **Coulomb + viscous only** | $F_s = F_c$ removes the Stribeck term; the level is constant                                                         |
| **Asymmetric friction**    | Separate parameters per direction of motion; handles direction-dependent drag                                        |
| **Karnopp model**          | Replaces the smooth layer with a velocity dead-zone and a stick state                                                |
| **Dahl / LuGre**           | Dynamic models with internal bristle state; capture presliding and hysteresis at the cost of state and a stiffer ODE |
| **Inverse-dynamics term**  | Same static model folded into a robot inverse-dynamics computation                                                   |

## Applications

- Robot joints and geared servo drives: cancelling joint friction in computed-torque and PD plus gravity controllers.
- Lead-screw and linear stages: reducing stick-slip in precision positioning.
- Gimbals and pan-tilt heads: smooth low-speed tracking.
- Torque or current-controlled actuators where friction dominates the low-speed error.

## Connections to Other Algorithms

| Algorithm                                             | Relationship                                                                                         |
|-------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| [Feedforward / 2-DOF](Feedforward2Dof.md)             | Same additive-feedforward structure; friction is a velocity-driven feedforward term                  |
| [PID Controller](Pid.md)                              | The feedback loop this term is added to; compensation reduces integrator work and stick-slip hunting |
| [Bang-Bang / Hysteresis](BangBangHysteresis.md)       | The $\varepsilon \to 0$ limit is a relay on velocity, with the same chattering mechanism             |
| [Saturation / Rate Limiter](SaturationRateLimiter.md) | The output clamp is the same saturation, applied to the compensation term alone                      |
| [QR Decomposition](../solvers/QrDecomposition.md)     | Backbone of the least-squares fit of the linear friction coefficients                                |

## References & Further Reading

- B. Armstrong-Hélouvry, P. Dupont, C. Canudas de Wit, "A survey of models, analysis tools and compensation methods for the control of machines with friction," *Automatica*, 30(7), 1994.
- H. Olsson et al., "Friction models and friction compensation," *European Journal of Control*, 4(3), 1998.

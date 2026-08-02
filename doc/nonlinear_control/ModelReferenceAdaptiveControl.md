# Model Reference Adaptive Control

## Overview & Motivation

Real hardware changes over time: motor resistance drifts with temperature, payload mass varies, actuators age. A fixed controller tuned at the factory cannot maintain performance across this variability. Model Reference Adaptive Control (MRAC) addresses this by continuously adjusting its own gains online, using only the tracking error between the real plant and a designer-specified reference model, until the plant's response matches the desired ideal. No knowledge of the exact plant parameters is needed — only the structure (the equations of motion) and the sign of the input gain.

This makes MRAC the canonical direct-adaptive scheme for the "known structure, unknown numbers" class of problems that appears throughout embedded control: a family of units sharing the same firmware but differing in parameter values, or a single unit whose parameters change slowly during operation.

## Mathematical Theory

### Problem Setup

Given a plant of the form

$$\dot{x} = a\,x + b\,u$$

where $a$ and $b$ are unknown constants with $b \neq 0$ and $\text{sgn}(b)$ known, the goal is to choose the control $u$ so that $x(t) \to x_m(t)$ as $t \to \infty$, where $x_m$ satisfies the reference model

$$\dot{x}_m = -a_m\,x_m + b_m\,r, \quad a_m > 0.$$

### Matching Conditions

The ideal control law that would make the plant identical to the reference model is

$$u^* = \theta_x^*\,x + \theta_r^*\,r$$

where

$$\theta_x^* = \frac{-(a_m + a)}{b}, \qquad \theta_r^* = \frac{b_m}{b}.$$

Since $a$ and $b$ are unknown, $\theta_x^*$ and $\theta_r^*$ cannot be computed directly; instead they are estimated online.

### Tracking Error Dynamics

Let $\hat{\theta}_x$ and $\hat{\theta}_r$ denote the current parameter estimates and define the parameter errors $\tilde{\theta} = \hat{\theta} - \theta^*$. The tracking error $e = x - x_m$ satisfies

$$\dot{e} = -a_m\,e + b\,(\tilde{\theta}_x\,x + \tilde{\theta}_r\,r).$$

### MIT Rule

The MIT rule minimises $J = \tfrac{1}{2}e^2$ by gradient descent on the parameter space:

$$\dot{\hat{\theta}}_x = -\gamma\,\text{sgn}(b)\,e\,x, \qquad \dot{\hat{\theta}}_r = -\gamma\,\text{sgn}(b)\,e\,r.$$

This is a first-order gradient update. It converges when $\gamma$ is small relative to the signal levels and the reference is persistently exciting. It has no global stability proof.

### Lyapunov Redesign

The Lyapunov method chooses the same update law but derives it from the Lyapunov function candidate

$$V(e, \tilde{\theta}) = \frac{e^2}{2} + \frac{|b|}{2\gamma}\left(\tilde{\theta}_x^2 + \tilde{\theta}_r^2\right).$$

Computing $\dot{V}$ and choosing the parameter update laws to make $\dot{V} \leq -a_m\,e^2 \leq 0$ yields precisely the same gradient form. The Lyapunov construction guarantees that $e$ and $\tilde{\theta}$ remain bounded for all $t \geq 0$ and that $e(t) \to 0$, even under large initial errors — a global stability guarantee that the pure MIT rule lacks.

### Discrete-Time Implementation

The continuous-time update is approximated by forward Euler with step $\Delta t$:

$$x_m[k+1] = x_m[k] + (A_m\,x_m[k] + B_m\,r[k])\,\Delta t$$

$$e[k] = x[k] - x_m[k+1]$$

$$u[k] = \hat{\theta}_x[k]\,x[k] + \hat{\theta}_r[k]\,r[k]$$

$$\hat{\theta}_x[k+1] = \hat{\theta}_x[k] - \gamma\,\text{sgn}(b)\,e[k]\,x[k]\,\Delta t$$

$$\hat{\theta}_r[k+1] = \hat{\theta}_r[k] - \gamma\,\text{sgn}(b)\,e[k]\,r[k]\,\Delta t$$

The same structure extends to the multi-input multi-output case using outer products of the error and regressor vectors, yielding matrix parameter estimates.

## Complexity Analysis

| Operation      | Time                           | Space                                     | Notes                                   |
|----------------|--------------------------------|-------------------------------------------|-----------------------------------------|
| ComputeControl | $O(n^2 + nm)$                  | $O(1)$ working registers                  | Reference model step + two outer products + two matrix-vector products |
| Memory         | $O(n^2 + nm)$ static           | Parameter matrices and reference state    | All fixed-size; no heap allocation      |

Here $n$ = StateSize and $m$ = InputSize. The dominant cost is the outer product update of the parameter matrices at each step.

## Step-by-Step Walkthrough

**First-order scalar example** ($n = m = 1$, $a = -1.5$, $b = 2$, $a_m = 1$, $b_m = 1$, $\gamma = 1$, $\Delta t = 0.01$):

1. At step $k=0$: $x = 2$, $r = 1$, $x_m = 0$, $\hat{\theta}_x = 0$, $\hat{\theta}_r = 0$.
2. Advance reference model: $x_m \leftarrow 0 + (-1 \cdot 0 + 1 \cdot 1) \cdot 0.01 = 0.01$.
3. Tracking error: $e = 2 - 0.01 = 1.99$.
4. Control output: $u = 0 \cdot 2 + 0 \cdot 1 = 0$ (initial parameters are zero).
5. Update $\hat{\theta}_x$: $0 - 1 \cdot (+1) \cdot 1.99 \cdot 2 \cdot 0.01 = -0.0398$.
6. Update $\hat{\theta}_r$: $0 - 1 \cdot (+1) \cdot 1.99 \cdot 1 \cdot 0.01 = -0.0199$.
7. At the next step the control becomes $u = -0.0398 \cdot x - 0.0199 \cdot r$, already driving the plant toward the reference model. After many steps, $\hat{\theta}_x \to \theta_x^* = 0.25$ and $\hat{\theta}_r \to \theta_r^* = 0.5$.

## Pitfalls & Edge Cases

- **Adaptation gain too large**: with the MIT rule, $\gamma$ large relative to signal power causes $\hat{\theta}$ to overshoot and the closed-loop to go unstable. The Lyapunov law is more forgiving but still requires reasonable $\gamma$.
- **Wrong sign of $b$**: if $\text{sgn}(b)$ is set incorrectly, adaptation drives parameters in the wrong direction and the error grows. This is a hard fault — the algorithm is designed around knowing the plant's sign.
- **No persistent excitation**: if $r$ is constant or bandlimited, $\hat{\theta}$ converges to some values that achieve tracking but not necessarily to $\theta^*$. The plant still tracks the reference model; only parameter identification fails.
- **Parameter drift**: in the presence of noise or unmodelled disturbances, $\hat{\theta}$ drifts even when $e$ is small. Standard remedies are $\sigma$-modification ($\dot{\hat{\theta}} = -\gamma\,e\,\phi - \sigma\,\hat{\theta}$) and parameter projection onto a compact set.
- **Euler discretisation error**: the Euler step introduces $O(\Delta t)$ error in the reference model trajectory and the parameter update. Small $\Delta t$ is needed for accuracy; a higher-order integrator (RK4) can be used for the reference model when $\Delta t$ is large.

## Variants & Generalizations

- **Indirect MRAC**: first identifies the plant parameters (via Recursive Least Squares) and then recomputes the matching gains; separates identification from control but requires a plant model structure assumption.
- **$\sigma$-modification**: adds a leakage term $-\sigma\,\hat{\theta}$ to the parameter update, bounding parameter drift at the cost of a small steady-state bias.
- **e-modification**: replaces $\sigma$ with $\sigma\,|e|\,\hat{\theta}$, so leakage is active only when the error is large and vanishes at steady state.
- **Parameter projection**: constrains $\hat{\theta}$ to a known compact set, preventing unbounded drift without degrading tracking.
- **Adaptive backstepping**: embeds MRAC-style parameter adaptation inside the Backstepping recursive design for strict-feedback plants with unknown parameters.
- **MRAC with reference model order > 1**: the same gradient law extends to any LTI reference model by tracking the full state vector $x_m \in \mathbb{R}^n$.

## Applications

- Motor drives with varying load inertia or resistance: MRAC maintains speed/position bandwidth despite parameter changes.
- Aerospace: autopilot adaptation to changing mass, fuel consumption, or aerodynamic coefficients.
- Robotic manipulators: payload-varying adaptive torque controllers.
- Power electronics: adaptive current controllers for converters with uncertain filter inductance.
- Embedded firmware for a product family: a single binary adapts to unit-to-unit hardware variation at startup.

## Connections to Other Algorithms

- **Backstepping Control**: MRAC handles unknown parameters in a fixed-structure plant; adaptive backstepping merges both, providing recursive stability proofs for uncertain strict-feedback systems.
- **Feedback Linearization**: cancels nonlinearities via an explicit model; MRAC is model-free in the parameter sense and is more robust when the model is uncertain.
- **Recursive Least Squares (RLS)**: the indirect-MRAC identification step; direct MRAC avoids explicit parameter estimation by updating control gains rather than plant parameters.
- **LQR**: optimal fixed-gain control for known linear plants; MRAC extends the design to plants with unknown parameters at the cost of the adaptation transient.
- **math::LinearTimeInvariant**: provides the reference model state-space container; the MRAC controller holds a reference to an LTI instance to step the reference trajectory.

## References & Further Reading

- K. J. Åström, B. Wittenmark, *Adaptive Control*, 2nd ed., Addison-Wesley, 1995.
- K. S. Narendra, A. M. Annaswamy, *Stable Adaptive Systems*, Prentice-Hall, 1989; Dover reprint 2005.
- S. Sastry, M. Bodson, *Adaptive Control: Stability, Convergence and Robustness*, Prentice-Hall, 1989.
- P. A. Ioannou, J. Sun, *Robust Adaptive Control*, Prentice-Hall, 1996 (free PDF available from the authors).

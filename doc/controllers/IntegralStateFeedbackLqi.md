# Integral State Feedback (LQI / Servo)

## Overview & Motivation

Plain LQR drives the system state toward the origin but leaves a persistent steady-state offset when a constant reference or disturbance is present. The integral state feedback controller
(LQI, or servo LQR) removes this offset by augmenting the plant with integrators on the
tracking error. A single LQR solve on the augmented system produces two gains: one that
acts on the physical states and one that closes the integral loop, guaranteeing zero
steady-state error to constant references with no manual trim.

## Mathematical Theory

### Plant Model

The discrete-time plant is

$$x_{k+1} = A x_k + B u_k, \quad y_k = C x_k$$

with state $x \in \mathbb{R}^n$, input $u \in \mathbb{R}^m$, and tracked output $y \in \mathbb{R}^p$.

### Augmented Plant

Define the integral-of-error state $x_i \in \mathbb{R}^p$:

$$x_{i,k+1} = x_{i,k} + (r_k - y_k) T_s$$

Stacking $\xi = [x^\top \; x_i^\top]^\top$ gives the augmented system

$$\xi_{k+1} = A_a \xi_k + B_a u_k + E_a r_k$$

$$A_a = \begin{bmatrix} A & 0 \\ -C T_s & I \end{bmatrix}, \quad B_a = \begin{bmatrix} B \\ 0 \end{bmatrix}, \quad E_a = \begin{bmatrix} 0 \\ T_s I \end{bmatrix}$$

### LQR Design on the Augmented Plant

Minimise the infinite-horizon quadratic cost

$$J = \sum_{k=0}^{\infty} \bigl(\xi_k^\top Q \xi_k + u_k^\top R u_k\bigr)$$

by solving the Discrete Algebraic Riccati Equation (DARE) for $P$:

$$P = A_a^\top P A_a - A_a^\top P B_a (R + B_a^\top P B_a)^{-1} B_a^\top P A_a + Q$$

The optimal gain partitions as $K_a = [K_x \mid K_i]$ where $K_x \in \mathbb{R}^{m \times n}$ acts
on the physical states and $K_i \in \mathbb{R}^{m \times p}$ acts on the integral states.

### Control Law

$$u_k = -K_x x_k - K_i x_{i,k}$$

At equilibrium $r - y = 0$, so $x_{i}$ stops changing, and the control law holds $y = r$ exactly.

## Complexity Analysis

| Case   | Time         | Space        | Notes                              |
|--------|--------------|--------------|------------------------------------|
| Design | $O((n+p)^3)$ | $O((n+p)^2)$ | DARE iteration on augmented system |
| Update | $O(m(n+p))$  | $O(p)$       | Two matrix-vector products         |

Design is a one-time offline computation. The per-sample update cost is dominated by the two
gain-state products and the integral accumulation.

## Step-by-Step Walkthrough

Consider a scalar plant ($n=1$, $m=1$, $p=1$, $T_s = 0.01$):

1. Form $A_a$ (2×2), $B_a$ (2×1) from plant matrices and $T_s$.
2. Choose $Q$ (2×2 diagonal) and $R$ (scalar) to weight states and input.
3. Solve DARE → $P$ (2×2) → $K_a = [k_x \; k_i]$ (1×2).
4. Per step: accumulate $x_i \mathrel{+}= (r - y) T_s$, output $u = -k_x x - k_i x_i$.
5. After ~200 steps the output converges to $r$ within numerical tolerance.

## Pitfalls & Edge Cases

- **Integral windup**: when the actuator saturates, the integral keeps growing because the
  control cannot reach the demanded value. Apply a clamp on $x_i$ or back-calculate
  (anti-windup) to prevent divergence after the saturation clears.
- **Marginally stable plant**: a plant with an open-loop integrator combined with the error
  integrator yields a double-integrator augmented system. The DARE still converges if the
  augmented pair $(A_a, B_a)$ is stabilisable; verify that condition before deploying.
- **Slow integral weighting**: under-weighting the integral state in $Q$ allows steady-state
  error to persist for many steps before correcting; over-weighting causes overshoot.
- **Sample-time mismatch**: the discrete integral $x_i$ accumulates $T_s$-scaled errors.
  Using the wrong $T_s$ at run time shifts the effective integral gain and breaks zero-error
  convergence.
- **Slow DARE convergence**: the augmented plant $(A_a, B_a)$ has eigenvalues at exactly 1, and at high sampling rates those eigenvalues approach 1 from many directions, requiring many more DARE iterations than the default 300. Raise the cap via the `MaxIterations` template parameter: `IntegralStateFeedbackLqi<float, StateSize, InputSize, OutputSize, 30000>`.

## Variants & Generalizations

- **Continuous-time LQI**: replace the discrete integrator with $\dot{x}_i = r - y$ and solve
  the continuous ARE.
- **Output-feedback LQI (LQGI)**: combine with a Kalman filter when only the output (not the
  full state) is measurable — the separation principle still holds.
- **Anti-windup**: add a saturation block on $x_i$ with back-calculation to recover from
  actuator limits without integral drift.
- **Multiple outputs**: the design extends directly to $p > 1$ by stacking $p$ integral states.

## Applications

- **Motor position/speed servo**: eliminate gravity or friction offsets without manual trim.
- **Process control**: temperature or pressure regulation with constant load disturbances.
- **Aerospace attitude control**: integral action compensates for persistent aerodynamic moments.
- **Robotics**: joint torque control with payload-induced constant forces.

## Connections to Other Algorithms

- **Lqr**: the base regulator; LQI delegates the DARE solve to `Lqr` on the augmented plant.
- **DiscreteAlgebraicRiccatiEquation**: the inner solver used by `Lqr`.
- **Lqg**: pairs `Lqr` with a Kalman filter; LQI could similarly pair with `Lqg` for
  output-feedback servo control.
- **Pid**: the integral channel of a PID is the scalar, single-output analogue of $K_i x_i$.
- **SaturationRateLimiter**: provides output clamping for anti-windup on the LQI integral.

## References & Further Reading

- B. D. O. Anderson, J. B. Moore, *Optimal Control: Linear Quadratic Methods*, Prentice Hall, 1990.
- G. F. Franklin, J. D. Powell, A. Emami-Naeini, *Feedback Control of Dynamic Systems*, 8th ed., Pearson, 2019. Chapter 9.
- K. J. Åström, B. Wittenmark, *Computer-Controlled Systems: Theory and Design*, 3rd ed., Prentice Hall, 1997. Chapter 5.

# Backstepping Control

## Overview & Motivation

Many embedded control plants are naturally cascaded: motor current drives torque, torque drives velocity, velocity drives position. Classical linear controllers treat the whole cascade as a single transfer function and tune gains empirically. Backstepping instead exploits the cascade structure directly, stabilising each integrator stage in sequence with a Lyapunov certificate attached at every step. The result is a constructive design recipe — not empirical tuning — that is provably stable by construction, which is essential for safety-critical electromechanical and flight control loops.

## Mathematical Theory

### Strict-Feedback Form

The controller targets plants in strict-feedback form:

$$\dot{x}_i = f_i(x_1,\ldots,x_i) + g_i(x_1,\ldots,x_i)\,x_{i+1}, \quad i = 1,\ldots,n-1$$

$$\dot{x}_n = f_n(x) + g_n(x)\,u$$

where $x_i \in \mathbb{R}$ are scalar states, $u \in \mathbb{R}$ is the real input, and $g_i \neq 0$ everywhere in the operating region (controllability condition).

### Stage Errors and Virtual Controls

Let $r(t)$ be the reference with known derivative $\dot{r}$. Define the stage-1 error:

$$z_1 = x_1 - r$$

To make $z_1$ converge, treat $x_2$ as a virtual control and choose the desired value:

$$\alpha_1 = \frac{\dot{r} - f_1 - k_1 z_1}{g_1}$$

so that $\dot{z}_1 = -k_1 z_1$ when $x_2 = \alpha_1$. Since $x_2$ cannot be instantaneously set, define the next error:

$$z_2 = x_2 - \alpha_1$$

Repeating down the chain, at stage $i$ the virtual control satisfies:

$$\alpha_i = \frac{\dot{\alpha}_{i-1} - f_i - k_i z_i - g_{i-1} z_{i-1}}{g_i}$$

The Lyapunov cross term $g_{i-1} z_{i-1}$ cancels the coupling between successive stages. At stage $n$, $\alpha_n$ is the actual input $u$.

### Lyapunov Certificate

The aggregate Lyapunov function:

$$V = \frac{1}{2} \sum_{i=1}^{n} z_i^2$$

has time derivative:

$$\dot{V} = -\sum_{i=1}^{n} k_i z_i^2 < 0 \quad \forall\, z \neq 0$$

This guarantees uniform asymptotic stability of the origin $z = 0$ (equivalently, $x \to r$) for all $k_i > 0$.

### Virtual Derivative Propagation

Each $\alpha_i$ depends on $x$, the gains, and the reference; its time derivative $\dot{\alpha}_i$ must be propagated analytically (chain rule) down the cascade. Numerical differentiation is excluded: noise amplification destroys the stability guarantee.

## Complexity Analysis

| Operation      | Time   | Space  | Notes                                      |
|----------------|--------|--------|--------------------------------------------|
| Construction   | $O(n)$ | $O(n)$ | Gain validation only                       |
| ComputeControl | $O(n)$ | $O(n)$ | Single forward sweep; one divide per stage |

All storage is in fixed-size arrays on the stack. No heap allocation, no recursion.

## Step-by-Step Walkthrough

**Order-2 chain of integrators** ($f_i = 0$, $g_i = 1$, $r = 0$, $\dot{r} = 0$):

1. $z_1 = x_1$. Stage-1 virtual control: $\alpha_1 = -k_1 z_1$.
2. $z_2 = x_2 - \alpha_1 = x_2 + k_1 x_1$.
3. $\dot{\alpha}_1 = -k_1 \dot{x}_1 = -k_1 x_2$ (propagated analytically).
4. Stage-2 actual input: $u = \dot{\alpha}_1 - k_2 z_2 - g_1 z_1 = -k_1 x_2 - k_2(x_2 + k_1 x_1) - x_1$.
5. Closed-loop: $\dot{V} = -k_1 z_1^2 - k_2 z_2^2 < 0$ — both errors decay independently at rates $k_1$ and $k_2$.

## Pitfalls & Edge Cases

- **Loss of controllability**: if $g_i(x) \to 0$, the virtual control $\alpha_i$ is undefined (division by zero). A guard asserts $|g_i| \geq \varepsilon$ and the controller holds its output rather than outputting infinity.
- **Derivative explosion**: at each stage $\dot{\alpha}_{i-1}$ must be the exact analytic derivative; finite-difference approximation introduces noise that worsens with stage depth and sampling rate.
- **Large gains**: high $k_i$ accelerates convergence but amplifies noise and can saturate actuators. In practice the gains are limited by actuator bandwidth and measurement noise.
- **Non-strict-feedback plants**: if $f_i$ depends on $x_j$ for $j > i$ (non-strict-feedback), the design does not directly apply; feedback linearisation or dynamic extension may be needed.

## Variants & Generalizations

- **Adaptive backstepping**: replaces known parameters in $f_i$ with online estimates (RLS or gradient update); bridges to Model Reference Adaptive Control.
- **Robust backstepping**: adds a sliding-mode or dead-zone term to each stage to handle bounded uncertainty in $f_i$ without cancellation.
- **Output-feedback backstepping**: combines with a high-gain observer to reconstruct unmeasured states before applying the control law.
- **Command-filtered backstepping**: replaces analytic derivative propagation with a first-order command filter, trading exact Lyapunov guarantees for implementability when $\dot{\alpha}$ is expensive to compute.

## Applications

- Motor drives: current → torque → speed → position cascade with known drift terms.
- UAV attitude and altitude control: thrust dynamics → angular rate → angle → position.
- Marine vessel path following: surge force → surge speed → horizontal position.
- Underactuated mechanical systems where the cascade structure is embedded in the Euler-Lagrange equations.

## Connections to Other Algorithms

- **Feedback Linearization**: cancels nonlinearities exactly via coordinate change; backstepping instead dominates them via stage-wise Lyapunov design — less sensitive to model error.
- **Sliding Mode Control**: achieves robustness via discontinuous switching; backstepping achieves it via constructive Lyapunov design without chattering.
- **Model Reference Adaptive Control (MRAC)**: adaptive backstepping extends this design to plants with unknown parameters, making the two approaches complementary.
- **LQR**: optimal for linear plants; backstepping generalises stability-guaranteed design to nonlinear strict-feedback plants at the cost of requiring the analytic model.

## References & Further Reading

- M. Krstić, I. Kanellakopoulos, P. Kokotović, *Nonlinear and Adaptive Control Design*, Wiley, 1995.
- H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice-Hall, 2002, Chapter 14.
- M. Krstić, P. Kokotović, "Control Lyapunov functions for adaptive nonlinear stabilization," *Systems & Control Letters*, 26(1), 1995.

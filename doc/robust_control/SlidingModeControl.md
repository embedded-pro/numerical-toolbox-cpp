# Sliding Mode Control

## Overview & Motivation

Variable-structure control systems change their structure depending on the current system state. Sliding Mode Control (SMC) is the most widely used instantiation: the controller switches between two or more structures to drive the state onto a designer-specified manifold — the **sliding surface** — and keep it there. Once constrained to the surface, the closed-loop dynamics are governed entirely by the surface geometry, independent of the plant model or matched disturbances. This structural robustness makes SMC a preferred choice for motor drives, DC-DC power converters, and any embedded plant whose parameters drift or are poorly known.

## Mathematical Theory

### State-Space Plant

The controller operates on a discrete-time linear plant

$$x_{k+1} = A x_k + B u_k, \quad x \in \mathbb{R}^n,\; u \in \mathbb{R}^m$$

where $A \in \mathbb{R}^{n \times n}$ and $B \in \mathbb{R}^{n \times m}$.

### Sliding Surface

A linear sliding surface is defined by

$$s(x) = S x, \quad S \in \mathbb{R}^{m \times n}$$

The sliding manifold $\{x : s(x) = 0\}$ is an $(n-m)$-dimensional subspace. The matrix $S$ is designed so the reduced-order dynamics on the manifold are stable and meet the desired closed-loop poles.

### Equivalent Control

The **equivalent control** $u_{eq}$ is the unique input that holds the state on $s = 0$ (i.e., $\dot{s} = 0$) for the nominal plant:

$$u_{eq} = -(S B)^{-1} S A x$$

This exists if and only if $S B$ is nonsingular, which is the relative-degree-one condition: each input channel must directly influence its corresponding sliding variable.

### Switching (Reaching) Term

The **switching term** adds a robust push toward the surface:

$$u_{sw} = (S B)^{-1} K \operatorname{sat}(s/\phi)$$

where $K \in \mathbb{R}^{m}$ is the per-channel switching gain and $\phi > 0$ is the **boundary-layer thickness**. The saturation function

$$\operatorname{sat}(\sigma) = \begin{cases} \sigma & |\sigma| \le 1 \\ \operatorname{sign}(\sigma) & |\sigma| > 1 \end{cases}$$

replaces the discontinuous $\operatorname{sign}(s)$ of ideal SMC with a continuous ramp inside $|s| \le \phi$, eliminating infinite-bandwidth chattering while maintaining the reaching property.

### Total Control Law

$$u = u_{eq} - u_{sw} = -(S B)^{-1}\bigl[S A x + K\,\operatorname{sat}(s/\phi)\bigr]$$

### Reaching Condition

The Lyapunov function $V = \tfrac{1}{2} s^\top s$ satisfies $\dot{V} < 0$ outside the boundary layer when $K_i > |d_i|$ for each matched-disturbance channel $d_i$. This guarantees finite-time arrival at $|s| \le \phi$.

### Sliding-Mode Dynamics

On the surface $s = 0$, the state evolves according to the $(n-m)$-dimensional reduced-order system. For a single-input system with $S = [c_1, \ldots, c_{n-1}, 1]$, the sliding pole is determined by the characteristic polynomial of the first $(n-1)$ rows under the surface constraint.

## Complexity Analysis

| Operation | Time | Space | Notes |
|-----------|------|-------|-------|
| Construction | $O(m^3 + n m)$ | $O(nm + m^2)$ | $(SB)^{-1}$ inversion once |
| ComputeControl | $O(n^2 + m^2)$ | $O(1)$ extra | $S A x$ dominates for large $n$ |
| Surface | $O(nm)$ | $O(1)$ extra | monitoring only |

All data is stored in fixed-size arrays; no heap allocation at any point.

## Step-by-Step Walkthrough

Consider a double-integrator plant ($n=2$, $m=1$):

$$A = \begin{bmatrix}0 & 1\\0 & 0\end{bmatrix}, \quad B = \begin{bmatrix}0\\1\end{bmatrix}$$

with surface $S = [1\; 1]$, gain $K = 2$, boundary layer $\phi = 0.05$.

1. **Precompute** $SB = 1$, so $(SB)^{-1} = 1$.
2. **At state** $x = [1, 0]^\top$: $s = 1 > \phi$, $\operatorname{sat}(s/\phi) = 1$.
3. **Equivalent control**: $u_{eq} = -(SA)x = -[0\;1][1\;0]^\top = 0$.
4. **Switching term**: $u_{sw} = 1 \cdot 2 \cdot 1 = 2$.
5. **Total**: $u = 0 - 2 = -2$. The large negative input decelerates the state toward the surface.
6. Once $|s| \le 0.05$, the saturation ramps linearly and the control becomes smooth.

## Pitfalls & Edge Cases

- **Singular $SB$**: the equivalent control is undefined. This occurs when the surface does not satisfy the relative-degree-one condition. Assert non-singularity at construction.
- **Chattering**: ideal SMC ($\phi \to 0$) switches at infinite frequency, exciting unmodeled dynamics and wearing actuators. The boundary layer is not optional for real hardware.
- **Gain too small**: if $K_i \le |d_i|$, the switching term cannot overcome the disturbance and the surface is never reached. The state remains bounded but does not converge.
- **Boundary-layer error**: the steady-state tracking error is $O(\phi)$. Reducing $\phi$ improves accuracy at the cost of higher-frequency control activity.
- **Discrete-time reaching**: the Zeno-like finite-time result holds in continuous time; in discrete time the state enters an $O(\phi + \Delta t)$ neighborhood of the surface.

## Variants & Generalizations

- **Higher-order SMC** (super-twisting): drives both $s$ and $\dot{s}$ to zero simultaneously, eliminating chattering without a boundary layer at the cost of requiring $\dot{s}$ estimates.
- **Terminal SMC**: uses a nonlinear surface $s = \dot{e} + \beta e^{p/q}$ to achieve finite-time convergence to the origin (not just to the surface).
- **Integral SMC**: augments the surface with an integral of the state to achieve zero steady-state error in the presence of constant disturbances.
- **Adaptive switching gain**: adjusts $K$ online to match the unknown disturbance bound, avoiding over-gain chattering.

## Applications

- Brushless motor current and speed control: fast switching, large disturbance rejection.
- DC-DC converters: inherent switching structure matches SMC's variable-structure nature.
- Pneumatic and hydraulic actuators: strong friction/backlash rejection via matched-disturbance cancellation.
- Satellite attitude control: robustness to inertia uncertainty and external torques.

## Connections to Other Algorithms

- **LQR**: smooth optimal alternative; minimises quadratic cost but has no guaranteed robustness to matched disturbances.
- **Disturbance Observer (DOB)**: estimates and cancels the disturbance algebraically; complementary to SMC for unmatched disturbances.
- **SaturationRateLimiter**: the boundary-layer `sat` function is semantically identical to the saturation block already in the library.
- **LuenbergerObserver**: may be combined with SMC when the full state is not measured (output feedback SMC).

## References & Further Reading

- V. Utkin, "Variable Structure Systems with Sliding Modes," *IEEE Transactions on Automatic Control*, 22(2), pp. 212–222, 1977.
- J.-J. Slotine, W. Li, *Applied Nonlinear Control*, Prentice-Hall, 1991, Chapter 7.
- H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice-Hall, 2002, Chapter 14.

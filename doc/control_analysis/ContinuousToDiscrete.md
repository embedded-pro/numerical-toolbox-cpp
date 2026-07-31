# Continuous-to-Discrete Conversion (c2d)

## Overview & Motivation

Physical plants and classical controllers are designed in continuous time using differential
equations and the Laplace transform. Microcontrollers, however, execute discrete update loops
at a fixed sampling period $T_s$. Continuous-to-discrete conversion bridges the two worlds: it
takes a continuous-time state-space model $(A, B, C, D)$ and produces the equivalent
discrete-time model $(A_d, B_d, C_d, D_d)$ that a sampled-data controller can execute directly.

Choosing the right discretization method is critical. A poor choice can push stable poles
outside the unit disk, introduce spurious resonances, or distort the DC gain — all of which
degrade closed-loop performance or cause instability on hardware.

## Mathematical Theory

### State-Space Representation

A continuous-time LTI system obeys

$$\dot{x}(t) = A\,x(t) + B\,u(t), \qquad y(t) = C\,x(t) + D\,u(t)$$

where $x \in \mathbb{R}^n$ is the state, $u \in \mathbb{R}^m$ the input, and $y \in \mathbb{R}^p$ the output.
The goal is to find $(A_d, B_d, C_d, D_d)$ such that

$$x_{k+1} = A_d\,x_k + B_d\,u_k, \qquad y_k = C_d\,x_k + D_d\,u_k$$

matches the continuous solution at sample instants $t_k = k\,T_s$.

### Zero-Order Hold (Van Loan's Trick)

Assuming the input is held constant between samples (true for a DAC or PWM), the exact solution is

$$A_d = e^{A T_s}, \qquad B_d = \left(\int_0^{T_s} e^{A\tau}\,d\tau\right) B$$

Van Loan (1978) observed that both quantities emerge from a single matrix exponential of the
augmented $(n+m) \times (n+m)$ block matrix:

$$M = \begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} T_s, \qquad
e^M = \begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix}$$

The upper-left $n\times n$ block is $A_d$; the upper-right $n\times m$ block is $B_d$.
$C$ and $D$ are unchanged under ZOH because the output equation is instantaneous.

### Tustin / Bilinear Transformation

The Tustin method replaces the Laplace variable via

$$s \;\longleftarrow\; \frac{2}{T_s}\,\frac{z-1}{z+1}$$

Setting $\alpha = 2/T_s$ and $P = (\alpha I - A)^{-1}$:

$$A_d = P(\alpha I + A), \quad B_d = 2P\,B, \quad C_d = \alpha\,C\,P, \quad D_d = D + C\,P\,B$$

Tustin maps the entire open left-half plane into the open unit disk, so it is
stability-preserving by construction. It is the preferred method for discretizing
controllers and filters when a specific corner frequency must be matched (after
frequency prewarping).

### Forward Euler

The cheapest approximation — first-order rectangle rule:

$$A_d = I + A\,T_s, \quad B_d = B\,T_s, \quad C_d = C, \quad D_d = D$$

Conditionally stable: poles may leave the unit disk when $T_s$ is too large relative to the
fastest mode.

### Backward Euler

Implicit first-order rule, equivalent to the $s \leftarrow (z-1)/(z\,T_s)$ substitution.
Setting $P = (I - A\,T_s)^{-1}$:

$$A_d = P, \quad B_d = P\,B\,T_s, \quad C_d = C\,P, \quad D_d = D + C\,P\,B\,T_s$$

Unconditionally stable (maps the left-half plane into the unit disk) but introduces phase lag
relative to the true system.

## Complexity Analysis

| Method         | Time         | Space                   | Notes                                       |
|----------------|--------------|-------------------------|---------------------------------------------|
| ZOH            | $O((n+m)^3)$ | $(n+m)^2$ augmented mat | One matrix exponential via scaling/squaring |
| Tustin         | $O(n^3)$     | $O(n^2)$                | One $n \times n$ LU factorisation           |
| Forward Euler  | $O(n^2)$     | $O(n^2)$                | Pure matrix multiply + add                  |
| Backward Euler | $O(n^3)$     | $O(n^2)$                | One $n \times n$ LU factorisation           |

All working memory is allocated on the stack; no heap is used.

## Step-by-Step Walkthrough

Consider a scalar integrator $\dot{x} = u$, so $A=0$, $B=1$, $C=1$, $D=0$, with $T_s = 0.1$.

**ZOH:**

$$M = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \times 0.1 = \begin{bmatrix} 0 & 0.1 \\ 0 & 0 \end{bmatrix}$$

$$e^M = I + M = \begin{bmatrix} 1 & 0.1 \\ 0 & 1 \end{bmatrix}$$

So $A_d = 1$, $B_d = 0.1$ — the integrator accumulates $u \cdot T_s$ each step.

**Tustin** ($\alpha = 20$):

$$P = (20 - 0)^{-1} = 0.05$$

$$A_d = 0.05 \times 20 = 1, \quad B_d = 2 \times 0.05 = 0.1$$

Both methods agree exactly for a pure integrator, as the bilinear transform maps the pole at
$s=0$ to $z=1$ without distortion.

## Pitfalls & Edge Cases

- **Singular matrix** — Tustin and Backward Euler require solving $(\alpha I - A)$ and
  $(I - A\,T_s)$, respectively. If $A$ has an eigenvalue exactly at $\alpha$ (Tustin) or
  $1/T_s$ (Backward Euler), the system matrix is singular. The LU factorisation will
  flag degeneracy; the caller should validate the sample rate.
- **Large $T_s$ with Forward Euler** — For a mode with time constant $\tau$, Forward Euler
  is stable only when $T_s < 2\tau$. Exceeding this causes poles to flip outside the unit
  circle and the discrete simulation diverges.
- **Frequency warping (Tustin)** — The Tustin transform compresses the entire imaginary
  axis into the unit circle, warping frequencies above $\pi/T_s$. To match a specific
  analogue frequency $\omega_0$, prewarp: use $\alpha = \omega_0 / \tan(\omega_0 T_s / 2)$
  instead of $2/T_s$.
- **Numerical conditioning** — For systems with eigenvalues spanning many decades, the
  augmented matrix exponential (ZOH) may suffer cancellation. Scaling the system before
  discretisation improves accuracy.

## Variants & Generalizations

- **Frequency prewarping** — Modify $\alpha = \omega_c / \tan(\omega_c T_s / 2)$ before
  Tustin to preserve the gain at a critical frequency $\omega_c$ exactly.
- **Generalised hold functions** — Higher-order holds (first-order hold, triangular hold)
  modify the ZOH integral; each requires its own augmented block.
- **Sampled-data $H_\infty$** — Discretisation under a minimax criterion rather than exact
  ZOH; accounts for inter-sample behaviour explicitly.

## Applications

- **Embedded control** — Convert a continuous PID or LQR design to a discrete update law
  running at a fixed ISR rate.
- **Digital filter design** — Convert an analogue prototype (Butterworth, Chebyshev) to
  a discrete IIR filter using Tustin with prewarping.
- **Simulation** — ZOH discretisation yields exact step-response simulation for piecewise-
  constant inputs, enabling hardware-in-the-loop testing without numerical ODE integration.

## Connections to Other Algorithms

- **MatrixExponential** — The ZOH method is entirely built on `expm` of the Van Loan
  augmented block; the exponential accuracy directly sets the ZOH accuracy.
- **LinearTimeInvariant** — Both input and output are `LinearTimeInvariant` structs;
  the discrete model drops straight into the state-update loop.
- **LU decomposition / Gaussian elimination** — Tustin and Backward Euler need one
  $n \times n$ factorisation each; an inline partial-pivoting LU is used to keep
  the module self-contained.

## References & Further Reading

- C. F. Van Loan, "Computing Integrals Involving the Matrix Exponential," *IEEE Transactions
  on Automatic Control*, 23(3), pp. 395–404, 1978.
- G. Franklin, J. D. Powell, M. L. Workman, *Digital Control of Dynamic Systems*, 3rd ed.,
  Addison-Wesley, 1997.
- C. Moler and C. Van Loan, "Nineteen Dubious Ways to Compute the Exponential of a Matrix,
  Twenty-Five Years Later," *SIAM Review*, 45(1), pp. 3–49, 2003.

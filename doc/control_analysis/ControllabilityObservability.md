# Controllability and Observability

## Overview & Motivation

Controllability and observability are fundamental structural properties of a linear dynamical system. They answer two essential design questions: can the inputs drive the state to any desired configuration, and can the initial state be inferred from the outputs alone?

A system lacking controllability has modes that no actuator can excite; a system lacking observability has modes that no sensor can detect. Both deficiencies make feedback design either impossible or degenerate. Checking these properties before attempting pole placement, LQR synthesis, or observer design prevents numerical failure and guides sensor-actuator placement.

## Mathematical Theory

### Prerequisites

Consider a discrete-time linear time-invariant system

$$x_{k+1} = A x_k + B u_k, \quad y_k = C x_k + D u_k$$

with state dimension $n$, input dimension $m$, and output dimension $p$.

### Controllability Matrix

The $n \times nm$ controllability matrix is

$$\mathcal{C} = \begin{bmatrix} B & AB & A^2 B & \cdots & A^{n-1}B \end{bmatrix}$$

The pair $(A, B)$ is controllable if and only if $\mathrm{rank}(\mathcal{C}) = n$.

### Observability Matrix

The $np \times n$ observability matrix is

$$\mathcal{O} = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1} \end{bmatrix}$$

The pair $(A, C)$ is observable if and only if $\mathrm{rank}(\mathcal{O}) = n$.

### Duality

Controllability and observability are dual: $(A, B)$ is controllable if and only if $(A^\top, B^\top)$ is observable. Equivalently, $\mathcal{C}(A, B) = \mathcal{O}(A^\top, C^\top)^\top$.

### Gramians

For a Schur-stable matrix $A$ (spectral radius $< 1$), the controllability and observability Gramians are the unique positive semi-definite solutions to the discrete Lyapunov equations

$$A W_c A^\top - W_c + BB^\top = 0, \quad A^\top W_o A - W_o + C^\top C = 0.$$

The Gramians encode more than rank: their eigenvalues quantify how easily each mode is excited or observed. Balanced truncation model reduction uses Gramians to identify modes that are simultaneously hard to reach and hard to see.

### Rank via Gaussian Elimination

Rank is computed by Gaussian elimination with partial pivoting on a working copy of the matrix. At each step the column with the largest absolute pivot is selected among remaining rows. A pivot is counted only if its magnitude exceeds $\tau \cdot \|M\|_\infty$ where $\tau$ is a user-supplied tolerance.

## Complexity Analysis

| Operation                      | Time                                              | Space          | Notes                |
|--------------------------------|---------------------------------------------------|----------------|----------------------|
| Controllability matrix         | $O(n^2 m)$ per column, $n$ columns → $O(n^3 m)$   | $O(nm)$ output | Iterative matmul     |
| Observability matrix           | $O(n^2 p)$ per row-block, $n$ blocks → $O(n^3 p)$ | $O(np)$ output | Iterative matmul     |
| Rank (Gaussian elim)           | $O(r^2 \min(r,c))$ where $r,c$ are matrix dims    | $O(rc)$ copy   | Partial pivoting     |
| Gramian (Lyapunov fixed-point) | $O(K n^3)$ iterations                             | $O(n^2)$       | Up to 200 iterations |

## Step-by-Step Walkthrough

Plant: $A = \begin{bmatrix}0 & 1 \\ -2 & -3\end{bmatrix}$, $B = \begin{bmatrix}0 \\ 1\end{bmatrix}$, $C = \begin{bmatrix}1 & 0\end{bmatrix}$.

**Controllability matrix:**

$AB = \begin{bmatrix}0 & 1 \\ -2 & -3\end{bmatrix}\begin{bmatrix}0 \\ 1\end{bmatrix} = \begin{bmatrix}1 \\ -3\end{bmatrix}$

$\mathcal{C} = \begin{bmatrix}0 & 1 \\ 1 & -3\end{bmatrix}$, $\det(\mathcal{C}) = -1 \neq 0$, rank $= 2$. Controllable.

**Observability matrix:**

$CA = \begin{bmatrix}1 & 0\end{bmatrix}\begin{bmatrix}0 & 1 \\ -2 & -3\end{bmatrix} = \begin{bmatrix}0 & 1\end{bmatrix}$

$\mathcal{O} = \begin{bmatrix}1 & 0 \\ 0 & 1\end{bmatrix}$, rank $= 2$. Observable.

## Pitfalls & Edge Cases

- **Numerical rank**: The binary controllable/observable answer is sensitive to the tolerance $\tau$. Near-rank-deficient systems (nearly uncontrollable modes) may flip with finite-precision arithmetic. Gramian eigenvalue magnitudes provide a graded measure.
- **Unstable A for Gramians**: The fixed-point iteration $X_{k+1} = AXA^\top + Q$ converges only when $\rho(A) < 1$ (Schur-stable). If $A$ has eigenvalues on or outside the unit disk the iteration diverges and a zero matrix is returned.
- **Wide controllability matrix**: When $m > 1$ the controllability matrix has more columns than rows; rank can be full even with fewer than $n$ block columns contributing independent directions.
- **Repeated eigenvalues**: A matrix with repeated eigenvalues can still be controllable; the Cayley-Hamilton theorem guarantees that $n$ block columns always suffice, but the matrix may be close to rank-deficient.

## Variants & Generalizations

- **Continuous-time Gramians**: satisfy $AW + WA^\top + BB^\top = 0$ and require a different solver (e.g., Bartels-Stewart or Hammarling).
- **PBH test**: The eigenvector-based Popov-Belevitch-Hautus test checks $\mathrm{rank}[A - \lambda I \;|\; B] = n$ for each eigenvalue $\lambda$, revealing which modes are uncontrollable.
- **Balanced realization**: Simultaneously diagonalizes $W_c$ and $W_o$ via similarity transformation; the resulting Hankel singular values rank-order modes by joint excitability and observability.
- **Stochastic observability**: Extends to systems driven by process noise; replaced by reachability Gramians.

## Applications

- Pre-synthesis checks before LQR, pole placement, or Luenberger observer design.
- Structural sensor/actuator placement: choose locations that maximize the minimum Gramian eigenvalue.
- Model order reduction: balanced truncation discards modes with small Hankel singular values.
- Fault detection: loss of observability of a mode may indicate sensor failure.

## Connections to Other Algorithms

- **LQR / DARE**: LQR requires controllability of $(A, B)$ and observability of $(A, Q^{1/2})$ for a finite-cost solution.
- **Luenberger Observer**: observer gain existence requires observability of $(A, C)$.
- **Lyapunov solver** (item 31): provides an exact algebraic solver for the Gramian equations, superseding the fixed-point iteration used here.
- **Balanced truncation**: uses both Gramians to compute Hankel singular values for model reduction.

## References & Further Reading

- K. J. Åström and R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton University Press, 2008, Ch. 8.
- C.-T. Chen, *Linear System Theory and Design*, 4th ed., Oxford University Press, 2013.
- R. Brockett, *Finite Dimensional Linear Systems*, SIAM, 2015.
- T. Kailath, *Linear Systems*, Prentice-Hall, 1980.

# Incremental QR Least Squares

## Overview & Motivation

Identification and calibration runs produce long logs — motor current against acceleration, joint
torque against motion — that a microcontroller cannot store. Accumulating the normal equations fits
in memory but squares the condition number, which single precision cannot afford. Real regressors
are also often rank-deficient: a parameter never shows up in the data, or only in a fixed
combination with another, and a plain solver then returns garbage.

Incremental QR least squares rotates each arriving row into a small triangular summary and forgets
it: memory stays fixed however long the log runs and the data matrix is never squared. The final
solve decomposes the summary into singular directions, inverts the strong ones, drops the weak ones,
and reports which parameter combinations the data could actually determine.

## Mathematical Theory

### Streaming QR Factor

Rows $(\mathbf{y}_k^T, \tau_k)$ with $\mathbf{y}_k \in \mathbb{R}^P$ arrive one at a time, $m$
unbounded. The task is $\min_{\boldsymbol{\pi}} \|Y\boldsymbol{\pi} - \boldsymbol{\tau}\|_2$ over the
stacked rows, taking the minimum-norm minimizer. Write
$Y = Q \begin{bmatrix} R \\ 0 \end{bmatrix}$ with $Q$ orthogonal and $R$ upper triangular
$P \times P$, and split $Q^T\boldsymbol{\tau} = (\mathbf{d}, \boldsymbol{\tau}_\perp)$. Orthogonal maps
preserve norms, so

$$\|Y\boldsymbol{\pi} - \boldsymbol{\tau}\|^2 = \|R\boldsymbol{\pi} - \mathbf{d}\|^2 + \rho,
\qquad \rho = \|\boldsymbol{\tau}_\perp\|^2$$

The problem lives in $R$, $\mathbf{d}$ and $\rho$, the part of the right-hand side no
$\boldsymbol{\pi}$ can reach: $P^2 + P + 2$ scalars with the row count. $Q$ is never formed.

**Row update.** Place the new row beneath $[\,R \mid \mathbf{d}\,]$. For $j = 1, \dots, P$, skip the
step if $y_j = 0$; otherwise rotate row $j$ of $R$ and the incoming row in their common plane:

$$\begin{bmatrix} c & s \\ -s & c \end{bmatrix}
\begin{bmatrix} R_{j,j:P} & d_j \\ y_{j:P} & \tau \end{bmatrix},
\qquad c = \frac{R_{jj}}{\ell},\; s = \frac{y_j}{\ell},\; \ell = \sqrt{R_{jj}^2 + y_j^2}$$

This zeroes $y_j$, leaves $R_{jj} = \ell \ge 0$ and touches only columns $j$ to $P$. After the sweep
the incoming row is zero in every column, so its leftover right-hand side $\tilde{\tau}$ reads
$0 \cdot \boldsymbol{\pi} = \tilde{\tau}$ and adds $\tilde{\tau}^2$ to $\rho$ whatever
$\boldsymbol{\pi}$ is. A zero pivot degenerates to a swap and a zero pair to the identity.

**Why not normal equations.** $Y^TY = R^TR$, yet the singular values of $R$ are those of $Y$: the
factor works at $\kappa(Y)$, the normal equations at $\kappa(Y)^2$. With single-precision
$\epsilon \approx 1.2 \times 10^{-7}$ they fail near $\kappa(Y) \approx \epsilon^{-1/2} \approx 3
\times 10^3$; the factor lasts to about $\epsilon^{-1}$.

### Rank-Revealing Solve

Take $R = U\Sigma V^T$ with $\sigma_0 \ge \dots \ge \sigma_{P-1} \ge 0$. In the coordinates
$\mathbf{z} = V^T\boldsymbol{\pi}$,

$$\|R\boldsymbol{\pi} - \mathbf{d}\|^2 = \sum_{i=0}^{P-1} \left(\sigma_i z_i - \mathbf{u}_i^T\mathbf{d}\right)^2$$

The rank is $r = \#\{\, i : \sigma_i > \text{tol} \cdot \sigma_0 \,\}$. For $i < r$ the term is zeroed by
$z_i = \mathbf{u}_i^T\mathbf{d} / \sigma_i$; for $i \ge r$ it cannot be removed and $z_i$ is free, so
the minimum-norm choice is $z_i = 0$. The discarded terms are charged to the residual:

$$\hat{\boldsymbol{\pi}} = \sum_{i < r} \mathbf{v}_i \, \frac{\mathbf{u}_i^T\mathbf{d}}{\sigma_i},
\qquad
\text{RMS} = \sqrt{\frac{\rho + \sum_{i \ge r} (\mathbf{u}_i^T\mathbf{d})^2}{m}}$$

The first $r$ rows of $V^T$ are the combinations $\mathbf{v}_i^T\boldsymbol{\pi}$ the data determine;
the rest are the unidentifiable directions. Moving $\boldsymbol{\pi}$ along them leaves the fit
unchanged, and $\hat{\boldsymbol{\pi}}$ is orthogonal to them.

Scaling a row and its right-hand side by $\sqrt{w_k}$ gives weighted least squares. Every step is
orthogonal, so row order changes the result only by rounding. With no rows there is no result; a
reset clears the state.

## Complexity Analysis

| Operation              | Time            | Space          | Notes                                                |
|------------------------|-----------------|----------------|------------------------------------------------------|
| Row, sparse (best)     | $O(P)$          | $P^2 + P + 2$  | Zero entries skipped; one short rotation per nonzero |
| Row, dense (worst)     | $O(P^2)$        | $P^2 + P + 2$  | $P$ rotations on $P - j + 1$ pairs, $P$ square roots |
| Solve                  | $O(P^3)$        | $O(P^2)$ extra | One $P \times P$ SVD, independent of $m$             |
| Whole stream, $m$ rows | $O(mP^2 + P^3)$ | $P^2 + P + 2$  | Memory does not grow with $m$                        |

At $P = 60$ the state is about 14 KB in single precision. Accumulation can run at the sample rate;
the solve is an offline or commissioning step. No heap, no recursion.

## Step-by-Step Walkthrough

**Full rank.** Rows $(1, x)$ at $x = (0, 1, 2, 3)$, $\boldsymbol{\tau} = (2.1, 4.9, 8.1, 10.9)$:

| Rows | $R_{00}$ | $R_{01}$ | $R_{11}$ | $d_0$  | $d_1$  | $\rho$ |
|------|----------|----------|----------|--------|--------|--------|
| 1    | 1        | 0        | 0        | 2.1    | 0      | 0      |
| 2    | 1.4142   | 0.7071   | 0.7071   | 4.9497 | 1.9799 | 0      |
| 3    | 1.7321   | 1.7321   | 1.4142   | 8.7180 | 4.2426 | 0.0267 |
| 4    | 2        | 3        | 2.2361   | 13     | 6.6188 | 0.0320 |

Row 1 meets a zero pivot and is adopted; row 2 rotates by $45^\circ$ and fits two points exactly;
from row 3 on, $\rho$ grows. With $\sigma \approx (4.1000, 1.0908)$, $r = 2$ and back-substitution
gives $\hat{\pi}_1 = d_1 / R_{11} = 2.96$, $\hat{\pi}_0 = (13 - 3 \cdot 2.96)/2 = 2.06$, with
RMS $= \sqrt{0.032 / 4} = 0.089443$.

**Rank-deficient.** Rows $(1, x, x)$ with $\boldsymbol{\tau} = 2 + 3x$: two equal columns leave the
third row of $R$ zero, $\rho = 0$, and $\sigma = (5.540482,\, 1.141517,\, 0)$, so $r = 2$. Only
$\pi_1 + \pi_2 = 3$ is determined; the minimum-norm split is $\hat{\boldsymbol{\pi}} = (2, 1.5, 1.5)$.
The last row of $V^T$ is the unidentifiable direction $(0, 1, -1)/\sqrt2$.

Verified against double-precision normal equations on random 3-parameter problems of 5 to 205 rows:
parameters agree to about $3 \times 10^{-6}$.

## Pitfalls & Edge Cases

- **Column scaling.** The rank test is relative to $\sigma_0$, so a column in large units hides the
  others. Scale columns to comparable magnitude before accumulating.
- **Tolerance.** Too small admits noise-level directions and the solution grows like $1/\sigma_i$; too
  large discards real ones, visible as a higher RMS. About $10^{-5}$ is a single-precision start.
- **RMS is not a noise estimate.** It divides by $m$, not $m - r$; scale its square by $m / (m - r)$
  for an unbiased variance.
- **Few or repeated rows.** Fewer rows than parameters is valid, with rank at most $m$; rows that
  repeat earlier ones leave the rank unchanged.
- **Long runs.** $\rho$ only grows; once it is about $2^{24}$ times one term, new terms vanish and the
  RMS turns optimistic. One NaN row poisons the state until a reset.

## Variants & Generalizations

- **Ridge.** Feed $P$ extra rows $\sqrt{\lambda}\,\mathbf{e}_j$ with zero right-hand side. Replacing
  $1/\sigma_i$ by $\sigma_i / (\sigma_i^2 + \lambda)$ filters smoothly instead of cutting off.
- **Forgetting.** Scaling $R$, $\mathbf{d}$ by $\sqrt{\lambda}$ and $\rho$ by $\lambda$ before each row
  gives exponentially weighted square-root recursive least squares.
- **Merging logs.** Rotating the rows of one summary $[\,R \mid \mathbf{d}\,]$ into another, and adding
  the $\rho$ values and counts, combines separately accumulated segments.

## Applications

- Inertial-parameter identification of robot arms, rank-deficient by construction
- Motor and drive parameter identification from long logs
- Friction identification: if the axis only ever moves one way, the sign column duplicates the offset
  column and the unidentifiable direction says so
- Sensor calibration from data streams too long to store

## Connections to Other Algorithms

- [Givens Rotation](../math/GivensRotation.md): the plane rotation that zeroes one entry per step.
- [Singular Value Decomposition](../solvers/SingularValueDecomposition.md): reveals rank and directions.
- [QR Decomposition](../solvers/QrDecomposition.md): the same factor, built row by row, no $Q$ kept.
- [Recursive Least Squares](RecursiveLeastSquares.md): forgetting and a $P \times P$ covariance; here
  all rows weigh equally and rank is revealed.
- [Linear Regression](LinearRegression.md): batch normal equations that store the samples.
- [Total Least Squares](TotalLeastSquares.md): also SVD-based, but for noisy regressors, not rank.
- [Friction Compensation](../controllers/FrictionCompensation.md): its Coulomb and viscous parameters
  are linear and identifiable this way.

## References & Further Reading

- G. H. Golub, C. F. Van Loan, *Matrix Computations*, 4th ed., Johns Hopkins University Press, 2013 —
  §5.1.8–5.3 (Givens QR, updating) and §5.5 (rank-deficient least squares).
- Å. Björck, *Numerical Methods for Least Squares Problems*, SIAM, 1996.

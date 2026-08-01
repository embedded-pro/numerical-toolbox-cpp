# Square-Root Kalman Filter

## Overview & Motivation

Standard Kalman filter implementations propagate the error covariance matrix $P$ directly. On
resource-constrained hardware — where floating-point arithmetic is limited to 32 bits — roundoff
errors accumulate with every predict-update cycle. After many iterations $P$ can lose symmetry
or positive-definiteness, causing the filter to diverge catastrophically.

The **square-root Kalman filter** eliminates this failure mode by never storing $P$ explicitly.
Instead it carries a Cholesky factor $S$ such that $P = S S^\top$, and rewrites every
predict/update operation as an orthogonal triangularization. Because orthogonal transforms
preserve matrix geometry exactly, the factor $S$ — and therefore the covariance $P = S S^\top$
— remains symmetric positive-definite by construction even in single-precision arithmetic.

A further benefit is numerical conditioning: $\mathrm{cond}(S) = \sqrt{\mathrm{cond}(P)}$, so
roughly twice as many significant bits survive across each filter step compared to the
conventional form.

## Mathematical Theory

### State-Space Model

The underlying discrete-time linear system is identical to the conventional Kalman filter:

$$x_k = F x_{k-1} + B u_{k-1} + w_{k-1}, \quad w \sim \mathcal{N}(0,Q)$$
$$z_k = H x_k + v_k, \quad v \sim \mathcal{N}(0,R)$$

The square-root filter parameterises uncertainty through lower-triangular factors
$S$, $S_Q$, $S_R$ satisfying $P = S S^\top$, $Q = S_Q S_Q^\top$, $R = S_R S_R^\top$.

### Predict Step (Time Update)

The goal is to compute $S^-$ such that $P^- = F P F^\top + Q = S^- (S^-)^\top$ without
forming $P$ or $P^-$.

Construct the $(2n \times n)$ pre-array:

$$\mathcal{A} = \begin{bmatrix} (F S)^\top \\ S_Q^\top \end{bmatrix}$$

Apply a QR decomposition $\mathcal{A} = Q_\perp R$ (orthogonal $Q_\perp$, upper-triangular $R$).
Then $S^- = R^\top$ is the new lower-triangular factor, because:

$$R^\top R = \mathcal{A}^\top \mathcal{A} = S^\top F^\top F S + S_Q^\top S_Q = F P F^\top + Q = P^-$$

### Update Step (Measurement Update)

Construct the $((m+n) \times (m+n))$ pre-array:

$$\mathcal{B} = \begin{bmatrix} S_R & H S \\ 0 & S \end{bmatrix}$$

Apply QR triangularization: $\mathcal{B} = Q_\perp \begin{bmatrix} S_y & \tilde{K} \\ 0 & S^+ \end{bmatrix}$

The block $S_y$ (upper-left, $m \times m$) is the innovation-covariance factor satisfying
$S_y S_y^\top = H P^- H^\top + R$. The block $S^+$ (lower-right, $n \times n$) is the
updated covariance factor. The Kalman gain emerges from the cross block:

$$K = \tilde{K}^\top S_y^{-\top}$$

solved via a triangular back-substitution rather than an explicit matrix inverse.

The state update is the standard correction:

$$x^+ = x^- + K(z - H x^-)$$

### QR via Givens Rotations

Both pre-arrays are triangularized by sequential Givens rotations applied column by column,
annihilating one sub-diagonal entry per rotation. This is cache-friendly, in-place, and
requires only $O(n^2)$ temporary storage — no heap allocation.

## Complexity Analysis

| Case    | Time           | Space    | Notes                                |
|---------|----------------|----------|--------------------------------------|
| Predict | $O(n^3)$       | $O(n^2)$ | Dominated by $F S$ multiply and QR   |
| Update  | $O((n+m) n^2)$ | $O(n^2)$ | QR on $(m+n) \times (m+n)$ pre-array |

where $n =$ `StateSize`, $m =$ `MeasurementSize`. All arrays are fixed-size; no heap is used.

## Step-by-Step Walkthrough

Consider a 1-state system ($n=1$, $m=1$) with $F=1$, $H=1$, $S=0.5$, $S_Q=0.1$, $S_R=0.3$:

**Predict:**

Pre-array: $\mathcal{A} = [0.5;\; 0.1]$ (column vector, already a column).
$R = \sqrt{0.5^2 + 0.1^2} = \sqrt{0.26} \approx 0.5099$.
New factor $S^- = 0.5099$, i.e. $P^- \approx 0.26$.

**Update** (measurement $z = 1.2$, prior $x^- = 1.0$):

Pre-array $\mathcal{B} = \begin{bmatrix}0.3 & 0.5099 \\ 0 & 0.5099\end{bmatrix}$.
One Givens rotation annihilates $\mathcal{B}_{10}=0$, yielding upper-triangular form.
$S_y \approx 0.5974$, $S^+ \approx 0.424$, $K \approx 0.741$.
$x^+ = 1.0 + 0.741 \times 0.2 = 1.148$.

## Pitfalls & Edge Cases

- **Factor sign convention.** The lower-triangular factor returned by Cholesky has positive
  diagonal; the QR step may flip signs. Diagonal entries should be taken as absolute values to
  maintain the canonical lower-triangular form.
- **Near-zero $R$.** When measurement noise approaches zero the innovation-covariance factor
  $S_y$ tends to zero; the triangular solve must be guarded against near-singular diagonals.
- **Near-zero $Q$.** With $S_Q = 0$ the predict step reduces the pre-array to a single block
  $(F S)^\top$; the QR degenerates to a transpose — no numerical issue, covariance only shrinks.
- **Large initial uncertainty.** A near-singular or large $P_0$ with $\mathrm{cond}(P_0) \approx 10^8$
  is handled safely because $\mathrm{cond}(S_0) \approx 10^4$, which 32-bit floats can represent.

## Variants & Generalizations

- **Information filter (dual form).** Propagate a factor of $P^{-1}$ instead of $P$.
  Cheaper when many measurements are fused per step; trivially encodes unknown initial state
  as zero information.
- **Square-root UKF.** Apply the same QR trick to the UKF sigma-point covariance propagation,
  replacing the weighted outer-product update with a rank-1 Cholesky update/downdate.
- **Potter / Carlson algorithms.** Older scalar measurement variants that process one
  measurement component at a time; simpler but restricted to diagonal $R$.

## Applications

- **Multi-sensor fusion on microcontrollers** — the primary motivation; 32-bit floats survive
  hundreds of filter steps without divergence.
- **INS/GNSS navigation** — long mission durations where a conventional filter would drift.
- **Safety-critical estimators** — provably PSD covariance at every step simplifies validation.
- **Battery state-of-charge estimation** — small $n$, constrained hardware, precision matters.

## Connections to Other Algorithms

- **Kalman Filter** — identical estimates; only the covariance bookkeeping differs. Use the
  conventional form when precision is not a concern and code simplicity is preferred.
- **Cholesky Decomposition** — used to convert a full covariance $P_0$ into the initial factor
  $S_0$ before constructing this filter.
- **QR Decomposition (Householder/Givens)** — the numerical engine of every predict/update step.
- **Unscented Kalman Filter** — a square-root variant exists (SR-UKF) that applies the same QR
  trick to the sigma-point covariance propagation.

## References & Further Reading

- P. Kaminski, A. Bryson, S. Schmidt, "Discrete Square Root Filtering: A Survey of Current
  Techniques," *IEEE Transactions on Automatic Control*, 16(6), 1971.
- R. van der Merwe and E. Wan, "The Square-Root Unscented Kalman Filter for State and
  Parameter-Estimation," *ICASSP*, 2001.
- G. Bierman, *Factorization Methods for Discrete Sequential Estimation*, Academic Press, 1977.

# Square-Root / Information Kalman Filter — Overview

## What it is
A numerically robust reformulation of the Kalman filter that never stores the covariance `P`
directly. Instead it propagates a **Cholesky/QR factor** `S` such that `P = S·Sᵀ` (the *square-root*
form) — or a factor of the inverse `P⁻¹` (the *information* form). The estimates are identical to a
textbook KF; only the bookkeeping changes.

## Why it matters (embedded)
On reduced-precision hardware a conventional Kalman filter can accumulate round-off until `P` loses
symmetry or positive-definiteness and the filter diverges. Working with a factor makes
`P = S·Sᵀ` **positive-definite by construction** and roughly **halves the condition number**, so the
same result survives in far fewer bits. That robustness is exactly what multi-sensor fusion on an
MCU or a safety-critical estimator needs.

## How it works (intuition)
Every predict/update is rewritten as an **orthogonal triangularization** (a QR/Givens step) of a
small stacked "pre-array". Orthogonal transforms preserve the covariance's geometry while returning
a fresh triangular factor — you get the new `S` without ever squaring numbers up into `F·P·Fᵀ`,
which is where a naive filter loses precision. The **information form** is the mirror image: it
carries a factor of `P⁻¹`, which is cheaper when you fuse many measurements at once and lets a
totally-unknown initial state be written simply as "zero information".

## Key parameters
- **S0 = chol(P0)** — initial covariance factor (the seeded uncertainty).
- **sqrtQ, sqrtR** — square-root factors of the process- and measurement-noise covariances.
- **F, H (, B)** — the usual state-space model matrices.
- **form** — square-root (covariance) vs information (inverse-covariance) duality.

## Reference
P. Kaminski, A. Bryson, S. Schmidt, "Discrete Square Root Filtering: A Survey of Current
Techniques," *IEEE Trans. Automatic Control*, 16(6), 1971.

## See also
`KalmanFilter` (the conventional covariance form), `CholeskyDecomposition` and `QrDecomposition`
(item 27, the numerical engine), `ExtendedKalmanFilter` (nonlinear extension).

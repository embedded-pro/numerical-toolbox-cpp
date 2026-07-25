# Continuous-to-Discrete (c2d) — Overview

## What it is
A converter that takes a continuous-time state-space model `(A, B, C, D)` — how a physical system
evolves in real time — and produces the discrete-time model `(A_d, B_d, C_d, D_d)` that a
sampled-data controller running at period `Ts` actually executes.

## Why it matters (embedded)
Plants and classical controllers are designed in continuous time (Laplace domain), but MCUs run
discrete update loops at a fixed sample rate. `c2d` is the bridge: design in continuous time, then
drop the result straight into the library's `LinearTimeInvariant` model for on-target execution.
Choosing the right method preserves stability and steady-state accuracy.

## How it works (intuition)
- **Zero-order hold (ZOH)** — assumes the input is held constant between samples (true for a DAC or
  PWM), giving the *exact* discrete model via a single matrix exponential of the augmented
  `[[A,B],[0,0]]` block (Van Loan's trick).
- **Tustin / bilinear** — the trapezoidal `s ↔ (2/Ts)(z−1)/(z+1)` substitution; warps the whole
  stable half-plane into the unit disk, so stability is preserved (best for controllers/filters).
- **Forward / backward Euler** — the cheapest rectangle-rule approximations, useful when `Ts` is
  tiny relative to the dynamics.

## Key parameters
- **Sample time `Ts`** — the discretization step; a smaller `Ts` shrinks method differences.
- **Method** — ZOH (exact for held inputs), Tustin (stability-preserving), Euler (cheapest).

## Reference
C. F. Van Loan, "Computing Integrals Involving the Matrix Exponential," *IEEE Trans. Automatic
Control*, 23(3), 1978; Franklin, Powell, Workman, *Digital Control of Dynamic Systems*.

## See also
`MatrixExponential` (item 29, the ZOH engine), `LinearTimeInvariant` (the model type produced),
`GaussianElimination` (the bilinear/Euler solves), `TransferFunctionStateSpace` (item 32).

# H∞ State-Feedback Control — Overview

## What it is
A robust optimal state-feedback law that minimizes the **worst-case** gain from disturbance to
performance error. Instead of assuming a known disturbance (like LQR/LQG), it assumes an adversary
picks the disturbance to hurt you the most, and designs the gain that best defends against it — the
guaranteed `L2` gain from disturbance `w` to error `z` is kept below a level `γ`.

## Why it matters (embedded)
When a loop must **not fail** under bounded but unknown model error or disturbance — actuators near
limits, safety-critical motion, plants that shift with load — H∞ gives a *provable* performance
bound rather than an average-case one. That guarantee is exactly what certification and
safety-critical embedded control ask for.

## How it works (intuition)
Frame the design as a two-player game: the controller minimizes error energy, a fictitious disturbance
maximizes it. The equilibrium is captured by a **game-theoretic Riccati equation** — the same algebraic
Riccati machinery as LQR, but with an *indefinite* weight that encodes the disturbance's antagonistic
role (a `−γ²` block). Solving it yields the feedback gain. A **bisection on `γ`** then squeezes the
attenuation level down to the smallest value for which a stabilizing solution still exists — the
sub-optimal `γ*`. As `γ` grows the adversary weakens and the whole thing gracefully becomes LQR.

## Key parameters
- **`γ` (attenuation level)** — the guaranteed disturbance-to-error `L2` gain; smaller is more robust and harder to achieve.
- **generalized plant `(A, B1, B2, C1, D12)`** — splits inputs into disturbance/control and defines the error channel.
- **bisection bounds & tolerance** — the search window for `γ*` and its resolution.

## Reference
J. Doyle, K. Glover, P. Khargonekar, B. Francis, "State-Space Solutions to Standard H₂ and H∞ Control
Problems," *IEEE Trans. Automatic Control*, 34(8), 1989.

## See also
`DiscreteAlgebraicRiccatiEquation` (the inner solve); `Lqr` (the `γ → ∞` limit);
`Lqg` (stochastic, average-case counterpart); `SlidingModeControl` (nonlinear robustness alternative).

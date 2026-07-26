# Runge-Kutta ODE Integrators — Overview

## What it is
A family of one-step methods that advance a differential equation `ẋ = f(x,u,t)` by sampling the
slope `f` at several points inside the step and combining them with fixed weights. **RK4** is the
classic fixed-step 4th-order workhorse; **Dormand-Prince (RK45)** is an *embedded* pair that returns
two solutions of different order so it can estimate its own error and adapt the step size.

## Why it matters (embedded)
It lets the device *simulate its own physics*: propagating the `dynamics/` models for prediction,
model-based control, and hardware-in-the-loop testing without a host. RK4 gives constant, fully
predictable work per tick — ideal for real-time loops.

## How it works (intuition)
A single Euler step trusts only the slope at the start of the interval; RK samples the slope again at
the midpoint and end, cancelling the low-order error terms. Dormand-Prince adds enough stages to
produce a 4th- and 5th-order estimate from the *same* samples; their difference is a cheap error
gauge that drives the step-size controller — shrink when the error is large, grow when the solution
is smooth.

## Key parameters
- **Step size `h`** — the accuracy-vs-cost knob for fixed-step RK4.
- **Tolerances (`absTol`, `relTol`)** — the target local error for the adaptive integrator.
- **`hMin` / `hMax`** — bounds that keep step-size control stable and bounded.

## Reference
J. R. Dormand, P. J. Prince, "A family of embedded Runge-Kutta formulae," *J. Comput. Appl. Math.*,
6(1), 1980; Hairer, Nørsett, Wanner, *Solving Ordinary Differential Equations I*.

## See also
`dynamics/` (right-hand sides), `ContinuousToDiscrete` (exact linear discretization),
`MatrixExponential`.

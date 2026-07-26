# Integral State Feedback (LQI / Servo) — Overview

## What it is
An LQR controller augmented with integral action. Extra states integrate the tracking error
`r − y`; an optimal gain is then designed for this *augmented* plant. The result is full-state
feedback that also drives steady-state error to exactly zero — the "servo" or LQI controller.

## Why it matters (embedded)
Plain LQR is a regulator: it drives the state to the origin, but leaves a steady-state offset
under constant references or disturbances (a load, gravity, a bias). Adding integral states removes
that offset with the same optimal machinery — no manual trim, no gain tweaking. It is the version
of LQR you actually deploy when the setpoint is not zero.

## How it works (intuition)
Append an integrator on the tracking error to the state vector. Because the integrator stops
changing only when `r − y = 0`, any nonzero steady-state error keeps pushing the control until the
error vanishes. Solving the LQR/Riccati problem on the augmented system yields two gains: one on
the physical states (`Kx`) and one on the accumulated error (`Ki`). The cost weights let you trade
how aggressively the integral closes the offset against overshoot.

## Key parameters
- **plant (A, B, C)** — the system to control.
- **Q, R weights** — LQR cost on the *augmented* state (including integral states) and input.
- **Ts** — sample time for the discrete error integration.
- **anti-windup limit** — bounds the integral when the actuator saturates.

## Reference
B. D. O. Anderson, J. B. Moore, *Optimal Control: Linear Quadratic Methods* (1990).

## See also
`Lqr` (the base design and Riccati solve); `DiscreteAlgebraicRiccatiEquation` (the gain solve);
`SaturationRateLimiter` (anti-windup on the output); `Pid` (the integral term, classical analogue).

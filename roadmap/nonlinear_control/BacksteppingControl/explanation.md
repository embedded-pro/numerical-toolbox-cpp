# Backstepping Controller — Overview

## What it is
A systematic, Lyapunov-based design method for **strict-feedback** nonlinear systems — cascades
where each state is driven by the next (`ẋ_i = f_i + g_i·x_{i+1}`) and only the last stage sees the
real input. Backstepping stabilizes the chain **one integrator at a time**, treating each
downstream state as a *virtual control* for the stage above it.

## Why it matters (embedded)
Many real plants are naturally cascaded: motor current → torque → velocity → position; thrust →
attitude → position. Backstepping gives a *constructive recipe* — no guesswork — that yields a
controller **with a stability proof attached**. For safety-critical electromechanical and flight
loops, "provably stable by construction" is worth far more than an empirically tuned PID.

## How it works (intuition)
Start at the output. Define the tracking error `z_1` and pretend the next state `x_2` is your knob:
pick the value `α_1` it *should* take to make `z_1` shrink. Of course `x_2` can't jump there
instantly, so define a new error `z_2 = x_2 − α_1` and repeat — now `x_3` is the virtual control.
March down the chain until you reach the real input `u` at the last stage. A single Lyapunov
function `V = ½·Σz_i²` accumulates every stage; making `V̇ < 0` at each step guarantees the whole
cascade converges. The catch: each virtual control's *derivative* `α̇` must be carried forward
analytically.

## Key parameters
- **strict-feedback model** — injected per-stage drift `f_i`, gain `g_i`, and virtual-control
  derivatives; the design assumes this structure.
- **stage gains k_i > 0** — one per integrator; each sets that stage's error-decay rate.
- **reference and its derivatives** — feedforward that the first stage tracks.

## Reference
M. Krstić, I. Kanellakopoulos, P. Kokotović, *Nonlinear and Adaptive Control Design* (Wiley, 1995).

## See also
`FeedbackLinearization` (cancels rather than dominates nonlinearities);
`ModelReferenceAdaptiveControl` (adaptive backstepping handles unknown parameters);
`SlidingModeControl` (robust alternative for matched uncertainty).

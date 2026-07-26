# Sliding Mode Control (SMC) — Overview

## What it is
A variable-structure controller that forces the state onto a designer-chosen **sliding surface**
`s(x) = 0` and keeps it there. Once on the surface, the closed-loop behaviour is governed entirely
by the surface geometry — not by the (uncertain) plant. A boundary layer around the surface tames
the switching so the control stays implementable.

## Why it matters (embedded)
SMC is **robust to matched disturbances and parameter uncertainty** by construction: as long as the
switching gain dominates the disturbance, the surface is reached regardless of load, friction, or
model error. That makes it a favourite in motor drives, DC-DC converters, and power electronics —
exactly the plants that live on microcontrollers and change under you.

## How it works (intuition)
Split the control into two parts. The **equivalent control** `u_eq` is the smooth term that would
hold the state on `s = 0` for the nominal plant. The **switching term** `−K·sat(s/φ)` adds a robust
push toward the surface whenever the state drifts off it. Ideal SMC uses `sign(s)`, which switches
infinitely fast (chattering); replacing it with a saturation over a thin boundary layer `φ` trades a
tiny steady-state error for a continuous, actuator-friendly signal.

## Key parameters
- **sliding surface `S`** — sets the reduced-order dynamics on the surface (the poles you keep).
- **switching gain `K`** — must exceed the disturbance/uncertainty bound to guarantee reaching.
- **boundary layer `φ`** — chattering-vs-accuracy knob; small `φ` = crisp but noisy, large `φ` = smooth but loose.

## Reference
V. Utkin, "Variable Structure Systems with Sliding Modes," *IEEE Trans. Automatic Control*, 22(2), 1977;
J.-J. Slotine, W. Li, *Applied Nonlinear Control* (1991).

## See also
`SaturationRateLimiter` (the boundary-layer clamp); `Lqr` (smooth optimal alternative);
`DisturbanceObserver` (add-on robustness for an existing loop).

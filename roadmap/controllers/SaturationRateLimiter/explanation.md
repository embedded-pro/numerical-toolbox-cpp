# Saturation / Rate-Limiter / Slew Blocks — Overview

## What it is
Two tiny actuator-constraint primitives and their composition. **Saturation** clamps a signal to
`[lo, hi]`. A **rate limiter** (slew limiter) caps how fast the signal may change per sample,
`|Δu| ≤ rate·Ts`. Together they model a real actuator that has both a travel limit and a maximum
speed.

## Why it matters (embedded)
Every real actuator — a valve, motor, heater, servo — has hard limits. Wrapping a controller
output in these blocks prevents commanding physically impossible moves and, critically, is the
**prerequisite for anti-windup**: the clamped value is what you feed back into an integrator so it
does not "wind up" while the actuator is pinned at its limit.

## How it works (intuition)
Saturation is a simple `min`/`max` sandwich. The rate limiter remembers its last output and
allows the new command to move at most `rate·Ts` toward the request each tick — a first-order
slew. Applying slew first and clamp second guarantees the emitted value respects both the speed
limit and the travel limits simultaneously.

## Key parameters
- **lo, hi** — output bounds (actuator travel range).
- **maxRate** — maximum change per second (slew rate).
- **sampleTime (Ts)** — controller period; sets the per-tick step `rate·Ts`.

## Reference
K. J. Åström, R. M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*
(2008), sections on actuator saturation and integrator windup.

## See also
`PidIncremental` (consumes the clamped output for anti-windup); `BangBangHysteresis` (a
different, discontinuous actuator constraint).

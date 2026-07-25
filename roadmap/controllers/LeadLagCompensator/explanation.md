# Lead-Lag Compensator — Overview

## What it is
A classical frequency-domain compensator, `C(s) = K·(s+z)/(s+p)`, discretized to a first-order
digital section. A **lead** network adds phase (speed, stability margin); a **lag** network adds
low-frequency gain (accuracy). The same one-pole/one-zero structure does both.

## Why it matters (embedded)
It is the workhorse of classical control: when a full state-space design is overkill, a lead or
lag compensator reshapes the loop's frequency response with just three coefficients and two state
words. Cheap enough for any MCU control loop, and every control engineer knows how to tune it from
a Bode plot.

## How it works (intuition)
A lead zero sits below its pole, so at mid frequencies the compensator's phase rises — that extra
phase, injected near crossover, increases phase margin and lets you push bandwidth higher. A lag
does the opposite: a high-frequency pole below the zero boosts low-frequency gain to kill
steady-state error while leaving the crossover region alone. The Tustin (bilinear) transform maps
the analog design to a discrete recurrence that preserves stability.

## Key parameters
- **K** — overall gain.
- **z (zero), p (pole)** — corner frequencies; their ordering picks lead vs lag.
- **Ts** — sample time used by the bilinear discretization.

## Reference
G. F. Franklin, J. D. Powell, A. Emami-Naeini, *Feedback Control of Dynamic Systems*
(lead/lag compensator design; bilinear transform).

## See also
`BiquadCascade` (second-order sections for higher-order compensators); `Pid` (a related
fixed-structure compensator); `FrequencyResponse` (to verify the shaped Bode plot).

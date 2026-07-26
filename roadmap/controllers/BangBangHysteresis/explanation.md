# Bang-Bang / Hysteresis (Relay) Controller — Overview

## What it is
A two-state on/off (relay) controller. The output snaps between two levels depending on whether
the measurement is above the upper threshold or below the lower threshold. The gap between the
two thresholds is a **Schmitt-trigger dead-band** that gives the relay memory.

## Why it matters (embedded)
This is the control law running in thermostats, fridge compressors, tank level switches, and
hysteretic power converters. It needs no gain tuning and no floating-point math — just two
comparisons — making it the cheapest closed-loop regulator that still behaves sensibly.

## How it works (intuition)
Without hysteresis, a plain comparator would chatter — switching rapidly whenever noise nudges
the signal across a single threshold. The dead-band fixes this: once the output goes High it
stays High until the signal falls all the way to the *lower* threshold, and vice-versa. The width
of the band directly sets the trade-off between regulation tightness and switching rate.

## Key parameters
- **lowThreshold / highThreshold** — the switch-off and switch-on points; their gap is the
  hysteresis band.
- **outputLow / outputHigh** — the two actuator levels (e.g. `0`/`1`, or `−1`/`+1`).

## Reference
K. J. Åström, R. M. Murray, *Feedback Systems* (2008), relay feedback; Ya. Z. Tsypkin,
*Relay Control Systems* (1984).

## See also
`SaturationRateLimiter` (continuous actuator constraints); relay feedback is also the basis of
Åström–Hägglund automatic PID tuning.

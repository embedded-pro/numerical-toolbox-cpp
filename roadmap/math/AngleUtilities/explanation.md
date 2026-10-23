# Angle Utilities — Overview

## What it is
Small helpers for angles that live on a circle: wrap any angle into a canonical interval, take the
shortest signed difference between two angles, and unwrap a wrapped measurement into a continuous
signal.

## Why it matters (embedded)
Angles wrap: a heading of 179° and one of −179° are 2° apart, not 358°. Controllers that subtract
angles naively command a full turn the wrong way; integrators that never wrap lose precision; encoders
and IMUs report wrapped values that estimators must unwrap. These bugs recur in every module that
touches angles, so one shared, tested implementation with one convention removes a whole class of
defects.

## How it works (intuition)
Shift the angle so the target interval starts at zero, take the remainder modulo a full turn, fix the
sign the remainder inherits from a negative input, and shift back. The shortest difference is the
wrapped plain difference. Unwrapping adds that shortest difference to the previous continuous value,
which is correct as long as consecutive samples move by less than half a turn.

## Key parameters
- **Interval convention** — `(−π, π]` for signed angles, `[0, 2π)` for unsigned ones.
- **Sample rate (unwrap)** — consecutive samples must differ by less than `π`.

## Reference
D. Goldberg, "What every computer scientist should know about floating-point arithmetic," *ACM
Computing Surveys*, 23(1), 1991 (range reduction and the cost of an inexact `2π`).

## See also
`ComplementaryFilter` (wraps its estimate), `Quaternion`, `SE3Transform` (#53, rotation-vector angles in
`[0, π]`).

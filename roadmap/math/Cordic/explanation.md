# CORDIC — Overview

## What it is
COordinate Rotation DIgital Computer: an iterative algorithm that computes trigonometric functions,
magnitudes, and vector rotations using only **shifts, adds, and a small angle table** — no hardware
multiplier required.

## Why it matters (embedded)
On FPU-less, multiplier-poor MCUs, `sin`/`cos`/`atan2`/`hypot` are expensive. CORDIC turns them into
a fixed sequence of shift-add steps with a **deterministic, data-independent cycle count** — exactly
what a hard real-time loop (FOC motor control, Cartesian↔polar conversion) needs.

## How it works (intuition)
Any rotation can be built by summing progressively smaller fixed rotations of `atan(2^-i)`, each of
which is just a shift-add because `tan = 2^-i`. In **rotation mode** you steer the residual angle to
zero and read off `sin`/`cos`; in **vectoring mode** you steer the `y` component to zero and read off
the angle (`atan2`) and the magnitude. A single constant gain `K` compensates for the per-step length
stretch.

## Key parameters
- **Iterations** — one extra bit of accuracy per iteration; sets the accuracy/latency trade-off.
- **Mode** — rotation (angle in → sin/cos out) vs vectoring (vector in → angle/magnitude out).
- **Gain `K ≈ 0.6073`** — the fixed prescale that removes the accumulated length growth.

## Reference
J. E. Volder, "The CORDIC Trigonometric Computing Technique," *IRE Trans. Electronic Computers*,
EC-8(3), 1959; R. Andraka, "A survey of CORDIC algorithms for FPGA-based computers," 1998.

## See also
`TrigonometricFunctions` (table-based alternative), `Quaternion` (axis-angle trig consumer).

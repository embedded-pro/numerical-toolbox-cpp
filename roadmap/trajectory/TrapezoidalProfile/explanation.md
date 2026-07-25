# Trapezoidal (LSPB) Velocity Profile — Overview

## What it is
A **linear-segment-with-parabolic-blends** (LSPB) motion profile: accelerate at a constant rate,
cruise at a constant velocity, then decelerate at a constant rate. The velocity-vs-time graph is a
trapezoid (or a triangle for short moves that never reach cruise speed).

## Why it matters (embedded)
It is the standard "get there as fast as the actuator allows" profile. Instead of fixing the
duration, you fix the **velocity and acceleration limits** and the profile computes the minimum
time that respects them — exactly what a motion controller needs to drive real hardware without
saturating drives.

## How it works (intuition)
Time splits into three phases with a simple symmetry: the accel and decel ramps are mirror images.
If the move is long enough, velocity saturates at `vMax` and coasts; if it is short, the profile
peaks below `vMax` and immediately decelerates — a triangle. A single distance test
(`d ≥ vMax²/aMax`) decides which case applies.

## Key parameters
- **`vMax`** — cruise velocity ceiling; caps the flat top of the trapezoid.
- **`aMax`** — ramp acceleration; sets the blend duration `vMax/aMax`.
- **Distance `qf − q0`** — decides trapezoid vs triangle and the total time.

## Reference
L. Biagiotti, C. Melchiorri, *Trajectory Planning for Automatic Machines and Robots* (2008),
Ch. 3 (trapezoidal / LSPB profiles).

## See also
`PolynomialTrajectory` (fixed-time, smooth), `SCurveProfile` (jerk-limited upgrade),
`SaturationRateLimiter` (online slew limiting).

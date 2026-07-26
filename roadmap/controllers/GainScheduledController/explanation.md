# Gain-Scheduled Controller — Overview

## What it is
A controller whose gains change with an operating condition. A lookup table stores a set of
pre-designed gains at several **breakpoints** of a scheduling variable (speed, load, altitude,
temperature); at run time the block interpolates between the nearest two sets.

## Why it matters (embedded)
Many "linear" plants are only linear near one operating point — a motor's dynamics change with
speed, an actuator's with load. Rather than redesign online (expensive, risky), engineers
pre-compute good gains at a grid of points offline and let the MCU interpolate. It extends proven
linear controllers to mildly nonlinear plants with almost no runtime cost.

## How it works (intuition)
Think of it as a piecewise-linear surface over the scheduling axis. Given the current scheduling
value, the block finds the bracketing interval, computes a blend weight, and linearly mixes the
two gain sets. Outside the table it holds the end gains (safe clamp-and-hold) rather than
extrapolating, which could produce unstable gains.

## Key parameters
- **breakpoints** — the scheduling-variable values where gains are defined (strictly increasing).
- **gain sets** — the controller gains stored at each breakpoint.
- **scheduling variable** — the measured operating condition selecting the gains.

## Reference
W. J. Rugh, J. S. Shamma, "Research on gain scheduling," *Automatica*, 36(10), 2000.

## See also
`Feedforward2Dof` (another way to handle varying operating conditions); `Lqr` (a common source of
the per-breakpoint gain sets).

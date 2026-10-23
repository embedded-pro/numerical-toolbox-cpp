# Motion Profiles

Time laws for point-to-point motion of a single axis: position, velocity, acceleration and jerk as functions of time, planned once and sampled every control tick.

## Algorithms

| Algorithm                                    | Description                                                                                                        |
|----------------------------------------------|--------------------------------------------------------------------------------------------------------------------|
| [Polynomial Profile](PolynomialProfile.md)   | Cubic (position + velocity) or quintic (+ acceleration) boundary-matched time law over a fixed duration            |
| [Trapezoidal Profile](TrapezoidalProfile.md) | Minimum-time trapezoid / triangle velocity profile (LSPB) and a fixed-duration plan for multi-axis synchronization |
| [S-Curve Profile](SCurveProfile.md)          | Seven-phase jerk-limited rest-to-rest profile with a closed-form three-step planner — continuous acceleration      |

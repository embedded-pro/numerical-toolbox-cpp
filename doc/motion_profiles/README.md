# Motion Profiles

Time laws for point-to-point motion of a single axis: position, velocity, acceleration and jerk as functions of time, planned once and sampled every control tick.

## Algorithms

| Algorithm                                                                 | Description                                                                                                                    |
|---------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
| [Polynomial Profile](PolynomialProfile.md)                                | Cubic (position + velocity) or quintic (+ acceleration) boundary-matched time law over a fixed duration                        |
| [Trapezoidal Profile](TrapezoidalProfile.md)                              | Minimum-time trapezoid / triangle velocity profile (LSPB) and a fixed-duration plan for multi-axis synchronization             |
| [Trapezoidal Profile from a Moving State](TrapezoidalProfileFromState.md) | Minimum-time trapezoid from the current position and velocity, separate acceleration and deceleration, for replanning mid-move |
| [S-Curve Profile](SCurveProfile.md)                                       | Seven-phase jerk-limited rest-to-rest profile with a closed-form three-step planner — continuous acceleration                  |
| [S-Curve Profile from a Moving State](SCurveProfileFromState.md)          | Seven-phase jerk-limited profile from the current position, velocity and acceleration; fixed-count bisection on the peak speed |

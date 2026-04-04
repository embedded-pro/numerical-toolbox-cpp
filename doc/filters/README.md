# Filters

Digital filtering algorithms for noise reduction, state estimation, and signal conditioning.

## Categories

### Active Filters

Filters that incorporate feedback and a dynamic internal model of the system.

| Algorithm                                                  | Description                                                              |
|------------------------------------------------------------|--------------------------------------------------------------------------|
| [Kalman Filter](active/KalmanFilter.md)                    | Optimal recursive state estimator for linear systems with Gaussian noise |
| [Extended Kalman Filter](active/ExtendedKalmanFilter.md)   | Nonlinear state estimator using first-order linearization (Jacobians)    |
| [Unscented Kalman Filter](active/UnscentedKalmanFilter.md) | Nonlinear state estimator using sigma points; no Jacobians required      |
| [Kalman Smoother](active/KalmanSmoother.md)                | Offline RTS smoother providing MMSE estimates over the full observation sequence |

### Passive Filters

*(Coming soon — FIR and IIR filter documentation)*

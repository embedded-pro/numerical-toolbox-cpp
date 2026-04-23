# Controllers

Feedback control algorithms for regulating dynamic systems in real time.

## Algorithms

| Algorithm                | Description                                                                             |
|--------------------------|-----------------------------------------------------------------------------------------|
| [PID Controller](Pid.md) | Proportional-Integral-Derivative controller using a discrete recursive formulation      |
| [LQR Controller](Lqr.md) | Linear Quadratic Regulator — optimal state-feedback control minimizing a quadratic cost |
| [LQG Controller](Lqg.md) | Linear Quadratic Gaussian — output-feedback optimal control via LQR + Kalman Filter     |
| [MPC Controller](Mpc.md) | Model Predictive Controller — receding-horizon optimal control with constraint handling |
| [Linear Time-Invariant Model](LinearTimeInvariant.md) | Discrete-time state-space plant model (A, B, C, D) shared across controllers and filters |

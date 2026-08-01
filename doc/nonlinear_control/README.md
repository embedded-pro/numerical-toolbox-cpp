# Nonlinear Control

Algorithms for nonlinear control design: controllers that exploit or cancel plant nonlinearities with stability guarantees.

## Algorithms

| Algorithm                                          | Description                                                                                                                                                                     |
|----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Backstepping Control](BacksteppingControl.md)     | Lyapunov-based recursive design for strict-feedback nonlinear cascades that stabilises each integrator stage in sequence, yielding a provably stable controller by construction |
| [Feedback Linearization](FeedbackLinearization.md) | Cancels a control-affine plant's known nonlinear dynamics via an inner control law, leaving decoupled integrator chains that a simple outer PD/LQR loop drives                  |

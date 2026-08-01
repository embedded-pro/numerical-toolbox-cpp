# Nonlinear Control

Algorithms for nonlinear control design: controllers that exploit a known plant model to cancel or structurally transform nonlinear dynamics.

## Algorithms

| Algorithm                                           | Description                                                                                                                                                   |
|-----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Feedback Linearization](FeedbackLinearization.md)  | Cancels a control-affine plant's known nonlinear dynamics via an inner control law, leaving decoupled integrator chains that a simple outer PD/LQR loop drives |

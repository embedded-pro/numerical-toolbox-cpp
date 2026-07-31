# Robust Control

Algorithms for robust control design: controllers that explicitly account for disturbances, uncertainty, and model mismatch.

## Algorithms

| Algorithm                                                             | Description                                                                                                                                                                    |
|-----------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Active Disturbance Rejection Control](ActiveDisturbanceRejection.md) | Near model-free controller pairing an Extended State Observer with bandwidth-parameterized PD feedback to estimate and cancel total disturbance in real time                   |
| [Sliding Mode Control](SlidingModeControl.md)                         | Variable-structure controller driving the state onto a sliding surface with a boundary layer to suppress chattering — robust to matched disturbances and parameter uncertainty |

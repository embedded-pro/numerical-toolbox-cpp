# Robust Control

Algorithms for robust control design: controllers that explicitly account for disturbances, uncertainty, and model mismatch.

## Algorithms

| Algorithm                                     | Description                                                                                                                                                                    |
|-----------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Sliding Mode Control](SlidingModeControl.md)         | Variable-structure controller driving the state onto a sliding surface with a boundary layer to suppress chattering — robust to matched disturbances and parameter uncertainty |
| [Disturbance Observer](DisturbanceObserver.md)        | Estimates lumped disturbance and model mismatch via the nominal plant inverse and a Q-filter, cancelling the disturbance to make the real plant behave like the nominal model  |

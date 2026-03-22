# Dynamics & Modeling

Rigid-body dynamics algorithms for robotics and mechanical systems.

## Algorithms

| Algorithm                                                 | Description                                                                                   |
|-----------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| [Articulated Body Algorithm](ArticulatedBodyAlgorithm.md) | $O(n)$ forward dynamics for serial chains — computes joint accelerations from applied torques |
| [Euler-Lagrange](EulerLagrange.md)                        | Forward and inverse dynamics via the Lagrangian formulation $M\ddot{q} + C\dot{q} + g = \tau$ |
| [Newton-Euler](NewtonEuler.md)                            | Single rigid-body forward and inverse dynamics using Newton's and Euler's equations           |
| [Recursive Newton-Euler](RecursiveNewtonEuler.md)         | $O(n)$ inverse dynamics for serial kinematic chains via recursive velocity/force propagation  |

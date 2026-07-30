# Math

Core mathematical primitives for numerical computation.

## Algorithms

| Algorithm                   | Description                                                                                   |
|-----------------------------|-----------------------------------------------------------------------------------------------|
| [CORDIC](Cordic.md)                                 | Iterative shift-add engine for sin/cos, atan2, magnitude, and vector rotation — no multiplier |
| [Quaternion](Quaternion.md)                         | Unit-quaternion rotation type: Hamilton product, SLERP, rotation-matrix and Euler conversions |
| [MatrixNorms](MatrixNorms.md)                       | Frobenius, 1-norm, infinity-norm on matrices; vector L2 norm/normalize                        |
| [Householder Transform](HouseholderTransform.md)    | Householder reflector for a sub-column — orthogonal, backward-stable factorization primitive  |
| [Givens Rotation](GivensRotation.md)                | Plane rotation zeroing one entry — streaming/sparse factorization primitive                   |
| [Triangular Solve](TriangularSolve.md)              | Upper-triangular back-substitution shared by Gaussian elimination and QR                       |
| [Step Response Metrics](StepResponseMetrics.md)     | Rise time, settling time, percent overshoot, peak time, and steady-state error from a bounded step-response vector |

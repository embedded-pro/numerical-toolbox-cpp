# Math

Core mathematical primitives for numerical computation.

## Algorithms

| Algorithm                                          | Description                                                                                                              |
|----------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------|
| [CORDIC](Cordic.md)                                | Iterative shift-add engine for sin/cos, atan2, magnitude, and vector rotation — no multiplier                            |
| [Quaternion](Quaternion.md)                        | Unit-quaternion rotation type: Hamilton product, SLERP, rotation-matrix and Euler conversions                            |
| [MatrixNorms](MatrixNorms.md)                      | Frobenius, 1-norm, infinity-norm on matrices; vector L2 norm/normalize                                                   |
| [Householder Transform](HouseholderTransform.md)   | Householder reflector for a sub-column — orthogonal, backward-stable factorization primitive                             |
| [Givens Rotation](GivensRotation.md)               | Plane rotation zeroing one entry — streaming/sparse factorization primitive                                              |
| [Triangular Solve](TriangularSolve.md)             | Lower/upper-triangular forward and back-substitution shared by Gaussian elimination, Cholesky, and QR                    |
| [Cholesky Decomposition](CholeskyDecomposition.md) | SPD factorization $A=LL^T$ with `Factor` (L) and `Solve` (SPD system) — twice as fast as LU                              |
| [Matrix Operations](MatrixOperations.md)           | Structural matrix utilities — `Symmetrize` (closest symmetric matrix)                                                    |
| [Step Response Metrics](StepResponseMetrics.md)    | Rise time, settling time, percent overshoot, peak time, and steady-state error from a bounded step-response vector       |
| [Matrix Exponential](MatrixExponential.md)         | Scaling-and-squaring with diagonal (6,6) Padé approximant — exact ODE solution operator and discretisation engine        |
| [Consistency Metrics](ConsistencyMetrics.md)       | NEES/NIS estimator consistency with χ² gates — normalised estimation error squared and normalised innovation squared     |
| [Angle Utilities](AngleUtilities.md)               | Wrap to $(-\pi, \pi]$ / $[0, 2\pi)$, shortest signed angle difference, and unwrapping of wrapped measurements            |
| [Tridiagonal Solver](TridiagonalSolver.md)         | Thomas algorithm — $O(n)$ factorization reused across right-hand sides for diagonally dominant / SPD tridiagonal systems |
| [Cubic Spline](CubicSpline.md)                     | Vector-valued $C^2$ cubic spline (clamped / natural ends) fitted with one shared tridiagonal factorization               |
| [SE(3) Transform](SE3Transform.md)                 | Rigid transforms with twist/wrench adjoints and an accurate exp/log (linear-first $(v;\omega)$ ordering)                 |

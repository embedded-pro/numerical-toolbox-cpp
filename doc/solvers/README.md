# Solvers

Numerical solvers for linear systems, polynomial roots, and matrix equations.

## Algorithms

| Algorithm                                                                  | Description                                                            |
|----------------------------------------------------------------------------|------------------------------------------------------------------------|
| [Cholesky Decomposition](CholeskyDecomposition.md)                         | Fast factorization for symmetric positive-definite matrices            |
| [Gaussian Elimination](GaussianElimination.md)                             | Direct solver for dense linear systems using partial pivoting          |
| [Levinson-Durbin](LevinsonDurbin.md)                                       | Fast solver for Toeplitz linear systems exploiting structural symmetry |
| [Durand-Kerner](DurandKerner.md)                                           | Simultaneous iterative root-finder for polynomials                     |
| [Discrete Algebraic Riccati Equation](DiscreteAlgebraicRiccatiEquation.md) | Iterative solver for the DARE arising in LQR and Kalman filter design  |
| [Runge-Kutta ODE Integrators](RungeKuttaIntegrators.md)                    | Fixed-step RK4 and adaptive Dormand-Prince RK45 for ODE integration    |
| [Spectral Radius](SpectralRadius.md)                                       | Dominant eigenvalue magnitude and discrete-time Schur stability margin |
| [Condition Number](ConditionNumber.md)                                     | 1-norm condition number estimate via column-wise inverse solve         |
| [QR Decomposition](QrDecomposition.md)                                     | Householder factorization and Givens streaming row update for least-squares solves |
| [LU Decomposition](LuDecomposition.md)                                     | PA = LU factorization with partial pivoting for general dense linear systems       |
| [Lyapunov / Sylvester Solvers](LyapunovSylvester.md)                       | Sylvester AX+XB=C and continuous/discrete Lyapunov solvers via Kronecker vectorisation |

# Discrete Algebraic Riccati Equation

## Mathematical Background

### The DARE

The Discrete Algebraic Riccati Equation (DARE) arises in optimal control and estimation problems for discrete-time linear systems. It computes a positive semi-definite matrix `P` that satisfies:

```
P = A' P A - A' P B (R + B' P B)⁻¹ B' P A + Q
```

where:
- `A` is the state transition matrix (n × n)
- `B` is the input matrix (n × m)
- `Q` is the state weighting matrix (n × n, positive semi-definite)
- `R` is the input weighting matrix (m × m, positive definite)
- `P` is the solution matrix (n × n, positive semi-definite)

### Iterative Solution

The implementation uses fixed-point iteration starting from `P₀ = Q`:

```
1. Compute  S      = R + B' P B
2. Solve    S * K  = B' P A        (via Gaussian elimination)
3. Update   P_new  = A' P A - A' P B * K + Q
4. Check    ||P_new - P_old|| < tolerance
5. Repeat until converged or MaxIterations reached
```

Each iteration refines `P` toward the steady-state solution. Convergence is guaranteed when the system `(A, B)` is stabilizable and `Q` is positive semi-definite.

### Connection to LQR

The DARE solution `P` directly yields the optimal LQR gain:

```
K = (R + B' P B)⁻¹ B' P A
```

The resulting control law `u[k] = -K x[k]` minimizes the infinite-horizon quadratic cost:

```
J = Σ (x[k]' Q x[k] + u[k]' R u[k])   for k = 0 to ∞
```

### Convergence

- The tolerance is type-aware via `math::Tolerance<T>()`, ensuring convergence is achievable for both floating-point and fixed-point types
- The algorithm is bounded by `MaxIterations` (default 100) to guarantee termination
- For well-conditioned systems, convergence typically occurs within 10–30 iterations

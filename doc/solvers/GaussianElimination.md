# Gaussian Elimination

## Mathematical Background

### The Linear System Problem

Gaussian elimination solves systems of linear equations of the form:

```
A * x = b
```

where:
- `A` is a square coefficient matrix (n × n)
- `x` is the unknown solution vector (n × 1)
- `b` is the right-hand side vector (n × 1)

### Algorithm

The algorithm consists of two phases: forward elimination and back-substitution.

#### Forward Elimination with Partial Pivoting

The matrix is transformed into upper triangular form by eliminating below-diagonal entries column by column. At each step, partial pivoting selects the row with the largest absolute value in the current column as the pivot row, improving numerical stability.

For each column `k`, the pivot row is found:

```
pivot_row = argmax |A[i][k]|   for i = k, ..., n-1
```

Then for each row `i > k`, the elimination factor and update are:

```
factor = A[i][k] / A[k][k]
A[i][j] = A[i][j] - factor * A[k][j]   for j = k, ..., n-1
b[i]    = b[i]    - factor * b[k]
```

After forward elimination, the system has the form:

```
[U] * x = b'
```

where `U` is upper triangular.

#### Back-Substitution

The solution is computed from the last row upward:

```
x[i] = (b'[i] - Σ U[i][j] * x[j]) / U[i][i]   for j = i+1, ..., n-1
```

### Computational Complexity

- **Time:** O(n³) for forward elimination, O(n²) for back-substitution
- **Space:** O(n²) — operates in-place on copies of the input

### Numerical Considerations

- Partial pivoting prevents division by very small numbers, reducing round-off error amplification
- The solver asserts that pivot elements are non-zero; singular or near-singular matrices will trigger a debug assertion
- For fixed-point types (Q15, Q31), intermediate computations must remain within the representable range [-1, 1)

### Multi-Column Systems

The `SolveSystem` function solves `A * X = B` where `B` has multiple columns by solving each column independently. This is used by the Discrete Algebraic Riccati Equation solver and the LQR controller to compute optimal gains.

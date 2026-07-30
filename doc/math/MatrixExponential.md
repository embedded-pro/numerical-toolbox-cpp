# Matrix Exponential

## Overview & Motivation

The matrix exponential is the fundamental solution operator for linear time-invariant (LTI) systems. Given a square matrix $A$, the matrix exponential $e^A$ maps initial conditions of the ODE $\dot{x} = Ax$ to their exact solution at unit time: $x(1) = e^A x(0)$. In embedded control, it appears as the exact discretisation of a continuous-time state-space model — turning the pair $(A_c, B_c)$ into the discrete $(A_d, B_d)$ that an MCU actually executes. Accurate, bounded computation of $e^A$ is therefore a prerequisite for correct control deployment.

## Mathematical Theory

### Series Definition

For a square matrix $A \in \mathbb{R}^{n \times n}$, the matrix exponential is defined by the power series

$$e^A = \sum_{k=0}^{\infty} \frac{A^k}{k!} = I + A + \frac{A^2}{2!} + \frac{A^3}{3!} + \cdots$$

The series converges for every finite matrix and every field of characteristic zero.

### Scaling and Squaring

Truncating the series directly is unreliable for large $\|A\|$ because the intermediate terms grow into a large "hump" before cancelling — a phenomenon of catastrophic cancellation. The scaling-and-squaring algorithm avoids this by exploiting the identity

$$e^A = \left(e^{A/2^s}\right)^{2^s}$$

Choose $s = \max\!\left(0, \lceil \log_2 \|A\|_\infty \rceil\right)$ so that $\|A/2^s\|_\infty \leq 1$, compute $e^{A/2^s}$ accurately on the shrunken argument, then recover $e^A$ by repeated squaring.

### Padé Approximant

For the scaled matrix $\hat{A} = A/2^s$, the exponential is approximated by the diagonal $(q, q)$ Padé rational function

$$e^{\hat{A}} \approx R_q(\hat{A}) = D_q(\hat{A})^{-1} N_q(\hat{A})$$

where $N_q$ and $D_q$ are matrix polynomials whose scalar counterparts are the numerator and denominator of the $(q,q)$ Padé approximant to $e^x$. The coefficients $c_k$ satisfy

$$c_k = \frac{(2q - k)!\, q!}{(2q)!\, k!\, (q-k)!}, \quad k = 0, 1, \ldots, q$$

For $q = 6$ the coefficients are $c_0 = 1$, $c_1 = \tfrac{1}{2}$, $c_2 = \tfrac{5}{44}$, $c_3 = \tfrac{1}{66}$, $c_4 = \tfrac{1}{792}$, $c_5 = \tfrac{1}{15840}$, $c_6 = \tfrac{1}{665280}$.

The even/odd split halves the number of matrix multiplications:

$$V = c_0 I + c_2 \hat{A}^2 + c_4 \hat{A}^4 + c_6 \hat{A}^6$$
$$U = \hat{A}\!\left(c_1 I + c_3 \hat{A}^2 + c_5 \hat{A}^4\right)$$
$$N_q = V + U, \quad D_q = V - U$$

The ratio $D_q^{-1} N_q$ is evaluated by solving the linear system $D_q X = N_q$ column-by-column using LU factorisation with partial pivoting, never forming $D_q^{-1}$ explicitly.

## Complexity Analysis

| Phase                     | Time        | Space      | Notes                                              |
|---------------------------|-------------|------------|----------------------------------------------------|
| Infinity-norm             | $O(n^2)$    | $O(1)$     | Determines scaling exponent $s$                    |
| Matrix powers $A^2, A^4, A^6$ | $3\,O(n^3)$ | $3 n^2$    | Even/odd split; three multiplications total        |
| Polynomial evaluation     | $O(n^3)$    | $2 n^2$    | Horner-style accumulation of $U$ and $V$           |
| LU solve ($D_q X = N_q$) | $O(n^3)$    | $n^2$      | One factorisation, $n$ substitution passes         |
| Squaring ($s$ steps)      | $s\,O(n^3)$ | $n^2$      | At most $\lceil \log_2 \|A\|_\infty \rceil$ steps |
| **Total**                 | $O((6+s)n^3)$ | $O(n^2)$ | Stack-only; no heap                                |

## Step-by-Step Walkthrough

Consider $A = \begin{bmatrix}0 & -1 \\ 1 & 0\end{bmatrix}$, a rotation generator with $\theta = 1$.

1. **Norm**: $\|A\|_\infty = 1$, so $s = 0$ (no scaling needed).
2. **Polynomial evaluation**: compute $\hat{A}^2 = -I$, $\hat{A}^4 = I$, $\hat{A}^6 = -I$. Then
   $$V = (c_0 - c_2 + c_4 - c_6)I, \quad U = (c_1 - c_3 + c_5)\hat{A}$$
3. **Padé ratio**: $R = D^{-1}N$ solved via LU.
4. **Squaring**: none ($s = 0$).
5. **Result**: matches $\begin{bmatrix}\cos 1 & -\sin 1 \\ \sin 1 & \cos 1\end{bmatrix}$ to within float tolerance.

## Pitfalls & Edge Cases

**Catastrophic cancellation without scaling**: naive Padé on a large argument produces entries that nearly cancel, magnifying rounding error. The scaling step ensures $\|\hat{A}\|_\infty \leq 1$ before the rational approximation is applied.

**Nilpotent matrices**: the series terminates in finitely many terms. Scaling-and-squaring handles this transparently; the Padé approximant reduces to a truncated polynomial.

**Stiff systems (large negative eigenvalues)**: the result is bounded because $e^{\lambda}$ with $\lambda \ll 0$ is near zero. Floating-point underflow may drive these entries to zero; this is physically correct and numerically harmless.

**Singular denominator**: $D_q$ is singular only if $e^A$ has a pole, which the matrix exponential never does ($A$ finite $\Rightarrow$ $e^A$ invertible). For the Padé denominator this means near-singularity can occur only at pathological arguments; the LU pivoting detects and gracefully handles it in practice.

**Scaling exponent overflow**: for a matrix with entries $\approx 10^{38}$ the exponent $s$ would be $\approx 126$, requiring 126 squarings. This is accepted behaviour; the algorithm remains correct but slow.

## Variants & Generalizations

**Higher-order Padé**: orders 8, 10, or 13 reduce the required scaling and improve accuracy for modest $\|A\|$. Higham's 2005 algorithm chooses the order adaptively. Order 6 is a reasonable default for embedded float arithmetic.

**Schur decomposition pre-conditioning**: computing $e^A = Q e^T Q^\mathsf{T}$ (with $T$ upper-triangular Schur form) avoids the hump phenomenon entirely and permits reuse of the Schur factors. The additional cost is the Schur decomposition itself ($O(n^3)$) and is not justified for small embedded matrices.

**Taylor series with Horner evaluation**: accurate only for $\|A\| \ll 1$; the scaling step achieves exactly this, making scaling-and-squaring a superset.

## Applications

- **Exact discretisation** (`ContinuousToDiscrete`): $A_d = e^{A_c \Delta t}$.
- **Continuous-time Gramians**: the controllability Gramian $W_c = \int_0^\infty e^{At} B B^\mathsf{T} e^{A^\mathsf{T}t}\,\mathrm{d}t$ requires $e^A$ repeatedly.
- **Lie-group integration**: for rigid-body dynamics the matrix exponential maps the Lie algebra $\mathfrak{so}(3)$ (skew-symmetric matrices) to the Lie group $SO(3)$ (rotation matrices).
- **Linear ODE simulation**: $x(t) = e^{At} x_0$ evaluated on an MCU for trajectory preview.

## Connections to Other Algorithms

`TriangularSolve` (`SolveUnitLowerTriangular`, `SolveUpperTriangular`) performs the back-substitution steps of the LU solve inside `SolvePade`. `MatrixNorms::InfinityNorm` computes the scaling exponent. `ContinuousToDiscrete` (roadmap item 30) is the primary consumer of this algorithm.

## References & Further Reading

- C. Moler, C. Van Loan, "Nineteen Dubious Ways to Compute the Exponential of a Matrix, Twenty-Five Years Later," *SIAM Review*, 45(1), 2003.
- N. J. Higham, "The Scaling and Squaring Method for the Matrix Exponential Revisited," *SIAM Journal on Matrix Analysis and Applications*, 26(4), 2005.
- N. J. Higham, "Functions of Matrices: Theory and Computation," SIAM, 2008, Chapter 10.

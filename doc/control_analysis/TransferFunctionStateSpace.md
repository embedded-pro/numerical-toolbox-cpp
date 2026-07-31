# Transfer-Function ↔ State-Space Conversion

## Overview & Motivation

Classical control design and modern state-space methods speak different mathematical languages. Frequency-domain tools — Bode plots, root locus, PID/lead-lag synthesis — operate on transfer functions, ratios of polynomials in the Laplace variable $s$. Modern methods — LQR, observers, Kalman filters — require the state-space quadruple $(A, B, C, D)$. This converter is the bridge. A compensator designed in the frequency domain can be dropped into a state-space runtime without hand-derivation, and a state-space plant model can be lifted into the frequency domain for classical analysis.

## Mathematical Theory

### Transfer Function

A SISO rational transfer function is

$$H(s) = \frac{b_0 s^n + b_1 s^{n-1} + \cdots + b_n}{a_0 s^n + a_1 s^{n-1} + \cdots + a_n}$$

Normalising by $a_0$ produces the monic denominator $s^n + \hat{a}_1 s^{n-1} + \cdots + \hat{a}_n$. When $\deg(\text{num}) = \deg(\text{den})$, a polynomial division peels off the direct feed-through scalar $D = b_0/a_0$, leaving a strictly proper remainder.

### Controllable Canonical Form

The monic denominator maps directly to the companion (controllable canonical) matrix

$$A = \begin{bmatrix} 0 & 1 & 0 & \cdots & 0 \\ 0 & 0 & 1 & \cdots & 0 \\ \vdots & & & \ddots & \vdots \\ -\hat{a}_n & -\hat{a}_{n-1} & \cdots & & -\hat{a}_1 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ \vdots \\ 0 \\ 1 \end{bmatrix}$$

The output row $C = \begin{bmatrix} \hat{b}_n' & \cdots & \hat{b}_1' \end{bmatrix}$ is formed from the strictly-proper remainder coefficients $\hat{b}_i'$ after the feed-through split.

### Observable Canonical Form

The observable canonical form is the algebraic dual of the controllable form:

$$A_\text{ocf} = A_\text{ccf}^\top, \quad B_\text{ocf} = C_\text{ccf}^\top, \quad C_\text{ocf} = B_\text{ccf}^\top, \quad D_\text{ocf} = D_\text{ccf}$$

Both realisations represent the same input-output map and share identical transfer functions.

### State-Space to Transfer Function: Faddeev–Le Verrier

Given $(A, B, C, D)$, the transfer function is recovered via

$$H(s) = C\,(sI - A)^{-1} B + D = \frac{C\,\text{adj}(sI - A)\,B + D\,\det(sI - A)}{\det(sI - A)}$$

The Faddeev–Le Verrier algorithm computes the characteristic polynomial $\det(sI - A) = s^n + c_1 s^{n-1} + \cdots + c_n$ and the adjugate action $\text{adj}(sI - A)B$ in a single recursion of $n$ steps:

$$M_0 = I, \quad c_k = -\frac{1}{k}\operatorname{tr}(A M_{k-1}), \quad M_k = A M_{k-1} + c_k I$$

The numerator coefficient for degree $n - k$ is $C M_k B$.

## Complexity Analysis

| Operation                 | Time     | Space    | Notes                                  |
|---------------------------|----------|----------|----------------------------------------|
| `ToControllableCanonical` | $O(n)$   | $O(n^2)$ | Direct coefficient placement           |
| `ToObservableCanonical`   | $O(n^2)$ | $O(n^2)$ | One transpose after CCF                |
| `ToTransferFunction`      | $O(n^4)$ | $O(n^2)$ | $n$ steps, each $O(n^3)$ matrix-matrix |

Memory is $O(n^2)$ for $A$ and $O(n)$ for coefficient arrays; all allocations are `std::array` on the stack.

## Step-by-Step Walkthrough

$H(s) = \dfrac{s + 2}{s^2 + 3s + 2}$, so $n = 2$, denominator $[1, 3, 2]$, numerator $[0, 1, 2]$.

**Monic normalisation:** already monic; $\hat{a}_1 = 3$, $\hat{a}_2 = 2$.

**Feed-through split:** $b_0 = 0$, so $D = 0$ and $\hat{b}' = [0, 1, 2]$.

**Companion matrix:**

$$A = \begin{bmatrix} 0 & 1 \\ -2 & -3 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}, \quad C = \begin{bmatrix} 2 & 1 \end{bmatrix}$$

**Observable form:**

$$A_\text{ocf} = \begin{bmatrix} 0 & -2 \\ 1 & -3 \end{bmatrix}, \quad B_\text{ocf} = \begin{bmatrix} 2 \\ 1 \end{bmatrix}, \quad C_\text{ocf} = \begin{bmatrix} 0 & 1 \end{bmatrix}$$

**Round-trip verification via Le Verrier ($n = 2$):**

- $M_0 = I$; $c_1 = -\tfrac{1}{1}\operatorname{tr}(AM_0) = -\operatorname{tr}(A) = 3$; $M_1 = A + 3I = \begin{bmatrix}3 & 1 \\ -2 & 0\end{bmatrix}$.
- $c_2 = -\tfrac{1}{2}\operatorname{tr}(AM_1) = -\tfrac{1}{2}\operatorname{tr}\!\begin{bmatrix}-2 & 0 \\ -6 & -2\end{bmatrix} = 2$.
- Numerator coefficients: $CM_1B = [2,1]\begin{bmatrix}1\\0\end{bmatrix} = 2$; $CM_2B = CM_0B = 0$.

Result: $H(s) = \dfrac{s + 2}{s^2 + 3s + 2}$. Identical to the original.

## Pitfalls & Edge Cases

- **Near-zero leading denominator coefficient:** the normalisation step divides by $a_0$; a guard is required to avoid division by zero.
- **Non-minimal realisation:** if numerator and denominator share a common factor (pole-zero cancellation), the realisation is valid but is not minimal — it is either uncontrollable or unobservable. The conversion does not detect or cancel common factors.
- **Le Verrier conditioning:** the recursion accumulates floating-point errors in $O(n^4)$ operations. For large $n$ the recovered transfer function coefficients degrade. Keep system orders to the small values typical on an MCU.
- **Proper vs strictly proper:** when $\deg(\text{num}) < \deg(\text{den})$ the feed-through term $D = 0$ and the split step is a no-op. No special branch is required; the leading numerator coefficient being zero makes $D = 0$ automatically.

## Variants & Generalizations

- **Phase-variable form:** an alternative state numbering; identical to controllable canonical form up to index reversal.
- **Modal canonical form:** diagonalises $A$; numerically sensitive but decouples modes for analysis.
- **Balanced realisation:** minimises the condition number jointly; requires Gramian computation and similarity transformation.
- **Continuous-time vs discrete-time:** the companion structure is identical; only the interpretation of $s$ vs $z$ changes.

## Applications

- Bridging frequency-domain compensator design to the `LinearTimeInvariant` runtime for embedded execution.
- Computing frequency response or root locus from a state-space plant model.
- Initialising Kalman filter or LQR designs from a transfer-function spec.
- Verifying a state-space model by recovering and inspecting its transfer function.

## Connections to Other Algorithms

- **`LinearTimeInvariant`:** the target realisation type; canonical forms drop directly into its `Step`/`Output` interface.
- **`ControllabilityObservability`:** a canonical realisation is minimal if and only if it is both controllable and observable; use these checks after conversion.
- **`FrequencyResponse` / `RootLocus`:** classical analyses that accept transfer-function coefficients; feed the recovered transfer function from `ToTransferFunction` directly.
- **`DurandKerner`:** factors the denominator polynomial into its roots (poles); combine with this converter for pole-zero plots.

## References & Further Reading

- T. Kailath, *Linear Systems*, Prentice-Hall, 1980 — canonical realisations and the state-space / transfer-function correspondence (Ch. 6).
- C.-T. Chen, *Linear System Theory and Design*, 4th ed., Oxford University Press, 2013.
- K. J. Åström and R. M. Murray, *Feedback Systems*, Princeton University Press, 2008, Ch. 9.

# Savitzky-Golay Filter

## Overview & Motivation

Many sensor signals — ECG, PPG, spectroscopy, vibration — carry narrow peaks whose height, width,
and area carry physical meaning. A boxcar moving average suppresses noise by treating the signal as
locally constant, but this assumption flattens and widens every peak. The Savitzky-Golay filter
replaces that assumption with a stronger one: the signal is locally a low-degree polynomial.
Fitting that polynomial to each window by least squares and evaluating it at the window centre
preserves peak shape far better while still attenuating high-frequency noise.

## Mathematical Theory

### Core Definitions

Let $x[n]$ be the input sequence. Define a symmetric window of length $W$ (odd) centred at sample
$n$, with half-width $h = (W-1)/2$. The window offsets are $j = -h, -h+1, \ldots, h$.

Fit a polynomial $p(j) = \sum_{d=0}^{P} a_d \, j^d$ of degree $P < W$ to the window samples
$\{x[n+j]\}$ in the least-squares sense. The output for the $D$-th derivative at the centre is:

$$y[n] = D! \, a_D$$

### Vandermonde and Normal Equations

Assemble the Vandermonde matrix $A \in \mathbb{R}^{W \times (P+1)}$ with
$A_{k,d} = j_k^{\,d}$, $j_k = k - h$. The least-squares coefficients solve:

$$(A^\top A)\,\mathbf{a} = A^\top \mathbf{x}$$

The projection matrix $C = (A^\top A)^{-1} A^\top$ is $(P+1) \times W$. Row $D$ of $C$, multiplied
by $D!$, gives the FIR convolution kernel $\mathbf{c} \in \mathbb{R}^W$:

$$y[n] = \sum_{k=0}^{W-1} c_k \, x[n - h + k]$$

Because $A$ depends only on $W$ and $P$, the kernel $\mathbf{c}$ is fixed and computed once.

### Smoothing Kernel ($D = 0$)

The centre row ($D = 0$) reproduces a polynomial fit evaluated at $j = 0$. For the standard
5-point, quadratic ($P = 2$) case the kernel is:

$$\mathbf{c} = \frac{1}{35}[-3, 12, 17, 12, -3]$$

DC gain is exactly 1; the kernel is symmetric, so the filter has **linear phase** with group delay
$h = (W-1)/2$ samples.

### Derivative Kernels ($D \geq 1$)

Row $D$ of $C$ times $D!$ yields the $D$-th smoothed derivative kernel. For $D = 1$ the output
approximates $\mathrm{d}x/\mathrm{d}n$ (in samples); multiply by $1/T_s^D$ to convert to physical
units.

## Complexity Analysis

| Case | Time       | Space      | Notes                                        |
|------|------------|------------|----------------------------------------------|
| All  | $O(W)$     | $O(W)$     | One multiply-accumulate per tap per sample   |
| Init | $O(P^3)$   | $O(P^2 W)$ | Constexpr kernel precomputed at compile time |

The runtime cost is $W$ multiply-adds — identical to a length-$W$ FIR filter. The $O(P^3)$
Gaussian elimination occurs entirely at compile time with zero runtime overhead.

## Step-by-Step Walkthrough

Window $W = 5$, polynomial order $P = 2$, derivative $D = 0$, half-width $h = 2$:

1. Offsets $j$: $-2, -1, 0, 1, 2$.
2. Build $A$ (5 rows, 3 cols): column 0 all ones; column 1 the offsets; column 2 the squares.
3. Compute $A^\top A$ (3 × 3 symmetric matrix).
4. Invert by Gaussian elimination to get $(A^\top A)^{-1}$.
5. Project: $C = (A^\top A)^{-1} A^\top$ (3 × 5 matrix).
6. Row 0 of $C$: $[-3, 12, 17, 12, -3] / 35$.
7. Each output is $c^\top$ times the 5-sample window.

Input $[0, 0, 0, 0, 1]$: output = $-3/35 \approx -0.086$ (transient). After the delay line fills
with a constant signal $v$, output = $v$ exactly.

## Pitfalls & Edge Cases

- **Startup transient**: the first $h$ outputs are produced from a partially filled window.
  Pre-fill the delay line with the expected initial value to suppress the transient.
- **Order equals $W - 1$**: the kernel becomes an interpolating polynomial; every window sample is
  reproduced exactly and noise suppression vanishes.
- **Narrow window, high order**: $P \geq W$ is forbidden; $P$ close to $W$ yields a near-singular
  $A^\top A$, degrading numerical accuracy.
- **Derivative scaling**: the kernel encodes $1/T_s^D$ implicitly only if $T_s = 1$ (samples).
  For physical units, multiply the output by $1/T_s^D$ at the call site.
- **Asymmetric / causal forms**: shifting row index selection in $C$ gives a causal kernel with
  reduced group delay at the cost of asymmetric (non-linear-phase) response.

## Variants & Generalizations

- **Causal SG filter**: use only past samples (shift window left); lower group delay, non-linear
  phase.
- **Higher-order smoothing**: increasing $P$ reduces bias on curved peaks but also reduces
  noise rejection — the classical bias-variance tradeoff.
- **2-D SG filter**: extend the Vandermonde to two spatial dimensions for image smoothing.
- **Adaptive window**: select $W$ based on local curvature to balance smoothing and peak fidelity.

## Applications

- ECG / PPG peak detection: preserve R-wave amplitude while suppressing baseline wander.
- Spectroscopy: smooth absorbance spectra without shifting or broadening absorption bands.
- Inertial measurement: smoothed angular velocity / acceleration from MEMS sensor streams.
- Process control: derivative term estimation from noisy sensor feedback.

## Connections to Other Algorithms

- `MovingAverage` is the $P = 0$ special case of Savitzky-Golay (constant polynomial); it
  maximises noise rejection but maximally distorts peaks.
- `Fir` with custom tap weights generalises both; Savitzky-Golay computes those weights
  analytically from least-squares polynomial fitting.
- `PolynomialFitting` (offline) performs the same projection without the sliding-window structure.
- `MedianFilter` suppresses impulsive spikes rather than Gaussian noise; the two are complementary.

## References & Further Reading

- A. Savitzky, M. J. E. Golay, "Smoothing and Differentiation of Data by Simplified Least Squares
  Procedures," *Analytical Chemistry*, 36(8), pp. 1627–1639, 1964.
- R. W. Schafer, "What Is a Savitzky-Golay Filter?," *IEEE Signal Processing Magazine*, vol. 28,
  no. 4, pp. 111–117, 2011.
- S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*, Ch. 14 —
  Introduction to Digital Filters. Available at dspguide.com.

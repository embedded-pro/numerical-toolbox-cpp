# Biquad / Second-Order-Section Cascade

## Overview & Motivation

High-order IIR filters realized as a single monolithic direct-form transfer function suffer
from severe numerical sensitivity: small perturbations in any coefficient can shift poles
dramatically, and round-off noise accumulates in proportion to the filter order. These
effects are catastrophic on fixed-point hardware and non-trivial even in single-precision
floating-point.

The solution adopted universally in professional DSP is to factor the transfer function into
quadratic terms — each with exactly two poles and two zeros — and chain them in series.
This cascade of second-order sections ("biquads") keeps each section's coefficients
well-scaled, its pole-sensitivity small, and its round-off noise bounded independently of
the total filter order.

## Mathematical Theory

### Transfer Function Factorization

Any real-coefficient rational transfer function of order $N$ factors as:

$$H(z) = G \prod_{k=1}^{\lceil N/2 \rceil} \frac{b_{0,k} + b_{1,k}z^{-1} + b_{2,k}z^{-2}}{1 + a_{1,k}z^{-1} + a_{2,k}z^{-2}}$$

Each factor is a second-order section (SOS). The overall frequency response is the product
of the per-section responses; the total group delay is the sum of per-section group delays.

### RBJ Cookbook Design Equations

For standard shelving, peaking, and pass/reject types, closed-form normalized coefficients
derive from the bilinear transform applied to an analog prototype. Define:

$$\omega_0 = \frac{2\pi f_c}{f_s}, \qquad c_\omega = \cos(\omega_0), \qquad \alpha = \frac{\sin(\omega_0)}{2Q}$$

**Low-pass:**

$$b_0 = \frac{1 - c_\omega}{2(1+\alpha)}, \quad b_1 = \frac{1-c_\omega}{1+\alpha}, \quad b_2 = b_0$$

$$a_1 = \frac{-2c_\omega}{1+\alpha}, \quad a_2 = \frac{1-\alpha}{1+\alpha}$$

**High-pass:** replace $(1 - c_\omega)$ with $(1 + c_\omega)$ and negate $b_1$.

**Notch (band-reject):**

$$b_0 = b_2 = \frac{1}{1+\alpha}, \quad b_1 = a_1 = \frac{-2c_\omega}{1+\alpha}, \quad a_2 = \frac{1-\alpha}{1+\alpha}$$

All coefficients are pre-normalized by $a_0 = 1 + \alpha$, so the feedback denominator leading
coefficient is always 1 and the hot path requires no division.

### Transposed Direct Form II (TDF-II)

TDF-II is the canonical embedded realization. It minimizes the number of state registers to
two per section while achieving low round-off noise. The recurrence for section $k$ is:

$$y[n] = b_0 x[n] + z_1[n-1]$$

$$z_1[n] = b_1 x[n] - a_1 y[n] + z_2[n-1]$$

$$z_2[n] = b_2 x[n] - a_2 y[n]$$

This is exactly five multiplies and four adds per sample — the theoretical minimum for an
arbitrary biquad — and requires no intermediate storage beyond $z_1$ and $z_2$.

## Complexity Analysis

| Case       | Time   | Space  | Notes                                           |
|------------|--------|--------|-------------------------------------------------|
| Per sample | $O(S)$ | $O(S)$ | $S$ = number of sections; 5 mults + 4 adds each |
| Design     | $O(1)$ | $O(1)$ | Closed-form RBJ formulas; no iteration          |
| Reset      | $O(S)$ | —      | Zero two state words per section                |

## Step-by-Step Walkthrough

**Example:** design a 4th-order low-pass as two cascaded 2nd-order sections.

Given $f_c = 100$ Hz, $f_s = 1000$ Hz, $Q = 0.707$ (Butterworth factor for each section):

1. Compute $\omega_0 = 2\pi \cdot 0.1 \approx 0.6283$
2. $c_\omega = \cos(0.6283) \approx 0.8090$, $\alpha = \sin(0.6283)/(2 \times 0.707) \approx 0.4142$
3. $a_0 = 1.4142$; normalize: $b_0 \approx 0.06745$, $b_1 \approx 0.13490$, $b_2 \approx 0.06745$,
   $a_1 \approx -1.1429$, $a_2 \approx 0.4128$
4. Use the same section twice to realize a 4th-order filter.
5. At each sample, pass input through section 1, then section 2: $y = H_2(H_1(x))$.

**Impulse response trace** for bypass section $\{b_0=1, b_1=b_2=a_1=a_2=0\}$:
Input $[1, 0, 0, \ldots]$ → output $[1, 0, 0, \ldots]$ — identity passthrough.

## Pitfalls & Edge Cases

- **Section ordering**: sections should be ordered by increasing peak gain and poles paired with
  their nearest zeros. Misordering can cause intermediate signals to overflow before later
  sections attenuate them, especially in fixed-point implementations.
- **Poles near the unit circle**: high-$Q$ or near-Nyquist designs push poles close to
  $|z| = 1$. Finite-precision rounding can move a pole just outside, causing instability.
  Use $Q \leq 30$ in single precision; double precision or lattice realizations for higher $Q$.
- **Denormal floats**: small state values approaching the denormal range stall the FPU pipeline
  on many embedded cores. Enabling flush-to-zero (FTZ) or the fast-math pragma prevents this
  at the cost of negligible numerical error.
- **DC gain normalization**: the RBJ low-pass has unity DC gain by construction. Gain-staging
  between sections is not required; each section's output is well-scaled relative to its input.

## Variants & Generalizations

- **Direct Form I**: maintains four state variables per section (input and output history) but
  provides extra dynamic range at the cost of higher memory. Preferred when signal levels are
  difficult to bound.
- **Lattice / wave-digital**: alternative topologies that remain stable under very aggressive
  coefficient quantization; used in high-speed fixed-point applications.
- **Parallel SOS**: sections are summed rather than chained; useful for multi-band equalizers.
- **Second-order allpass**: $b_0 = a_2$, $b_1 = a_1$, $b_2 = 1$ — phase rotation without
  magnitude change, used in crossover networks and polyphase systems.

## Applications

- Multi-pole anti-aliasing and reconstruction filters in ADC/DAC chains.
- Audio equalization (parametric EQ, shelving filters) in embedded audio processors.
- Vibration isolation and sensor conditioning in industrial control systems.
- ECG/EEG baseline wander removal with high-pass SOS cascades.
- Motor drive current-sensing loop compensation when a precise roll-off characteristic is needed.

## Connections to Other Algorithms

- **IIR (direct form)**: the single-section IIR is the primitive building block. A cascade of
  biquads is simply a structured composition of single sections with superior numerical properties.
- **NotchCombFilter**: the notch filter is a single biquad specialized to place zeros exactly
  on the unit circle; a cascade realizes multi-pole notch designs.
- **IirFilterDesign**: a companion design routine generates SOS coefficient arrays from an analog
  prototype via the bilinear transform, feeding directly into the cascade.
- **RecursiveBuffer**: the IIR simulator uses `math::RecursiveBuffer` for time-domain state;
  the biquad avoids the overhead by keeping only two explicit state scalars per section.

## References & Further Reading

- R. Bristow-Johnson, "Cookbook formulae for audio EQ biquad filter coefficients",
  <https://www.w3.org/TR/audio-eq-cookbook/>
- A. V. Oppenheim & R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Ch. 6
  (cascade and parallel structures).
- R. G. Lyons, *Understanding Digital Signal Processing*, 3rd ed., Prentice Hall, 2011, Ch. 6.
- S. J. Orfanidis, *Introduction to Signal Processing*, Prentice Hall, 1996, Ch. 12
  (second-order sections and ladder filters).

# Frequency Response

## Overview & Motivation

The frequency response $H(e^{j\omega})$ characterizes how a discrete-time linear system (filter) modifies the amplitude and phase of each frequency component in a signal. Computing and plotting the frequency response (Bode plot) is essential for verifying filter designs, understanding system behavior, and tuning parameters. This implementation evaluates the transfer function at logarithmically spaced frequencies, producing magnitude (dB) and phase (degrees) curves.

## Mathematical Theory

### Transfer Function Evaluation

Given a filter with feedforward coefficients $\{b_i\}_{i=0}^{P-1}$ and feedback coefficients $\{a_i\}_{i=0}^{Q-1}$, the frequency response at angular frequency $\omega$ is:

$$H(e^{j\omega}) = \frac{\sum_{i=0}^{P-1} b_i \cdot e^{-j\omega i}}{\sum_{i=0}^{Q-1} a_i \cdot e^{-j\omega i}}$$

For FIR filters, the denominator reduces to $a_0 = 1$.

### Magnitude and Phase

From the complex-valued response:

$$|H(e^{j\omega})| = \sqrt{\text{Re}(H)^2 + \text{Im}(H)^2}$$

$$\text{Magnitude (dB)} = 20 \log_{10} |H(e^{j\omega})|$$

$$\text{Phase (degrees)} = \frac{180}{\pi} \cdot \arg(H(e^{j\omega}))$$

### Frequency Grid

Frequencies are evaluated on a logarithmic grid from $f_s / N$ to $f_s / 2$ (Nyquist), where $f_s$ is the sampling frequency and $N$ is the number of evaluation points. The angular frequency at each point is:

$$\omega_k = \frac{2\pi f_k}{f_s}$$

## Complexity Analysis

| Case    | Time             | Space            | Notes                              |
|---------|------------------|------------------|------------------------------------|
| Average | $O(N \cdot (P+Q))$ | $O(N)$        | $N$ frequency points, $P+Q$ coefficients |

## Step-by-Step Walkthrough

Given a simple first-order low-pass FIR filter with $b = [0.5, 0.5]$, $a = [1.0]$, and $f_s = 100$ Hz, evaluated at $f = 10$ Hz:

1. Compute $\omega = 2\pi \cdot 10 / 100 = 0.2\pi$
2. Numerator: $0.5 \cdot e^{0} + 0.5 \cdot e^{-j0.2\pi} = 0.5 + 0.5(\cos(0.2\pi) - j\sin(0.2\pi))$
   - $= 0.5 + 0.5(0.809 - j0.588) = 0.905 - j0.294$
3. Denominator: $1.0$
4. $|H| = \sqrt{0.905^2 + 0.294^2} = 0.952$
5. Magnitude: $20\log_{10}(0.952) = -0.43$ dB
6. Phase: $\arctan(-0.294/0.905) = -18°$

## Pitfalls & Edge Cases

- **Zero denominator**: If the denominator evaluates to zero at a frequency, the implementation sets it to 1.0 to avoid division by zero. This produces a meaningful (if approximate) result near resonances.
- **QNumber conversion**: When coefficients are in Q15/Q31 format, they are converted to float via `math::ToFloat()` before evaluation. This ensures sufficient dynamic range for the complex arithmetic.
- **Logarithmic spacing**: Very low frequencies near DC may have sparse coverage. The lowest evaluated frequency is $f_s / N$.
- **Phase wrapping**: The `std::arg` function returns phase in $(-\pi, \pi]$. Phase unwrapping is not performed.

## Variants & Generalizations

- **Linear frequency spacing**: Alternative to logarithmic spacing for uniform frequency resolution.
- **Zero-pole form**: Evaluate directly from pole-zero locations rather than polynomial coefficients.
- **Chirp z-transform**: Generalization that evaluates the z-transform along arbitrary paths in the z-plane.

## Applications

- Verifying filter coefficient designs match the desired specification
- Bode plot generation for stability analysis
- Comparing filter topologies (FIR vs. IIR, different orders)
- System identification validation

## Connections to Other Algorithms

- **FIR / IIR Filters**: The frequency response evaluates the transfer functions implemented by `Fir` and `Iir` filters.
- **FFT**: The DFT of a zero-padded impulse response produces the frequency response at uniformly spaced frequencies. This class generalizes to arbitrary frequency points.
- **Root Locus**: Both frequency response and root locus analyze the same transfer function from complementary perspectives (frequency domain vs. pole-zero domain).
- **Power Density Spectrum**: PDS estimates the spectral content of a signal; frequency response characterizes the system that shapes it.

## References & Further Reading

- Oppenheim, A.V. and Schafer, R.W., *Discrete-Time Signal Processing*, 3rd ed., Pearson, 2009 — Chapters 5–6.
- Proakis, J.G. and Manolakis, D.G., *Digital Signal Processing: Principles, Algorithms, and Applications*, 4th ed., Pearson, 2006 — Chapter 6.
- Smith, S.W., *The Scientist and Engineer's Guide to Digital Signal Processing*, California Technical Publishing, 1997 — Chapter 21.

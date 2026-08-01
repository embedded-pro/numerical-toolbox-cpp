# Discrete Wavelet Transform (Haar / Daubechies)

## Overview & Motivation

The Discrete Wavelet Transform (DWT) decomposes a finite-length signal into a hierarchy of
approximation and detail coefficients at multiple scales. Unlike the DFT, which projects onto
infinitely-supported sinusoids, wavelets are compactly supported: they are simultaneously
localised in time and frequency. This dual localisation makes the DWT the canonical tool for
detecting transients, edges, and non-stationary features — phenomena that a pure frequency
representation blurs across the entire spectrum.

In embedded condition-monitoring and edge-ML pipelines, the DWT is valued for three properties:
its $O(N)$ complexity (a factor of $\log_2 N$ faster than the FFT), its ability to concentrate
signal energy in very few coefficients (sparsity), and the perfect-reconstruction guarantee that
lets processed coefficients be inverted exactly.

## Mathematical Theory

### Quadrature-Mirror Filter Bank

A single DWT stage passes a length-$N$ sequence $x[n]$ through two complementary finite-impulse-
response filters, then downsamples each output by two.

$$
c_A[i] = \sum_{k=0}^{P-1} h[k]\, x[2i + k \bmod N], \qquad
c_D[i] = \sum_{k=0}^{P-1} g[k]\, x[2i + k \bmod N], \qquad i = 0,\ldots,\tfrac{N}{2}-1
$$

where $h$ is the scaling (low-pass) filter with $P$ taps and $g$ is the wavelet (high-pass) filter.
For an orthogonal wavelet the two filters satisfy the **quadrature-mirror** relation:

$$g[k] = (-1)^k\, h[P - 1 - k]$$

This guarantees that the two sub-bands together cover the full bandwidth without overlap or gap.

### Multilevel Decomposition (Mallat's Algorithm)

The approximation $c_A$ is fed back into the same filter pair. After $L$ levels the coefficient
array contains $L$ detail bands and one coarse approximation:

$$
\text{coeffs} = [\underbrace{c_D^{(1)}}_{N/2},\; \underbrace{c_D^{(2)}}_{N/4},\; \ldots,\; \underbrace{c_D^{(L)}}_{N/2^L},\; \underbrace{c_A^{(L)}}_{N/2^L}]
$$

The layout is critical: level $\ell$ detail starts at offset $\sum_{j=1}^{\ell-1} N/2^j$ within
the coefficient buffer, and the final approximation occupies the last $N/2^L$ positions.

### Perfect Reconstruction

The synthesis (inverse) stage upsamples each sub-band by two and applies the dual filter pair
$(h_r, g_r)$. For orthogonal wavelets the synthesis filters are time-reversals of the analysis
filters:

$$h_r[k] = h[P - 1 - k], \qquad g_r[k] = g[P - 1 - k]$$

Combined, analysis and synthesis satisfy $H(z)H_r(z^{-1}) + G(z)G_r(z^{-1}) = 2$, the
alias-cancellation and distortion-free conditions, giving exact reconstruction:

$$\hat{x}[n] = x[n]$$

to floating-point rounding.

### Parseval Identity (Energy Preservation)

For an orthogonal wavelet the DWT is a unitary transform:

$$\sum_{n=0}^{N-1} x[n]^2 = \sum_{n=0}^{N-1} c[n]^2$$

where $c$ contains all detail and approximation coefficients.

### Wavelet Families

**Haar** ($P = 2$): $h = [1,\, 1]/\sqrt{2}$. Piecewise-constant basis; discontinuous; simplest
possible.

**Daubechies-2** ($P = 4$, denoted db2 or D4): coefficients chosen so the wavelet has two
vanishing moments — it annihilates linear trends. This produces smoother reconstructions and
better energy compaction for smooth signals.

## Complexity Analysis

| Case    | Time   | Space  | Notes                                        |
|---------|--------|--------|----------------------------------------------|
| Forward | $O(N)$ | $O(N)$ | Geometric series: $N + N/2 + \cdots \leq 2N$ |
| Inverse | $O(N)$ | $O(N)$ | Identical pass through synthesis bank        |

Memory is one length-$N$ coefficient buffer, two length-$N/2$ working arrays, and the static
filter table of size $P$ — no heap allocation.

## Step-by-Step Walkthrough

Input: $x = [1, 2, 3, 4]$, Haar wavelet, $L = 1$.

**Stage 1 (analysis):**

| $i$ | $2i$ | $2i+1$ | $c_A[i] = (x[2i]+x[2i+1])/\sqrt{2}$ | $c_D[i] = (x[2i]-x[2i+1])/\sqrt{2}$ |
|-----|------|--------|-------------------------------------|-------------------------------------|
| 0   | 0    | 1      | $(1+2)/\sqrt{2} \approx 2.121$      | $(1-2)/\sqrt{2} \approx -0.707$     |
| 1   | 2    | 3      | $(3+4)/\sqrt{2} \approx 4.950$      | $(3-4)/\sqrt{2} \approx -0.707$     |

**Coefficient buffer** (layout detail | approx):

$$\text{coeffs} = [-0.707,\; -0.707,\; 2.121,\; 4.950]$$

**Reconstruction:** upsample $c_A$ and $c_D$, apply synthesis filters, add:

$$\hat{x} = [1.0,\; 2.0,\; 3.0,\; 4.0] \checkmark$$

## Pitfalls & Edge Cases

- **$N$ must be divisible by $2^L$.** Odd-length or non-aligned signals at deeper levels produce
  fractional half-lengths and are rejected by design.
- **Boundary handling must match.** Periodic (modulo) extension is used for both analysis and
  synthesis; mixing it with symmetric extension on the other pass destroys perfect reconstruction.
- **Filter length vs. signal length.** At each level the signal halves; once it equals $P$ the
  periodic convolution wraps completely. Stop decomposition before the signal shorter than the
  filter length to avoid artefacts.
- **Fast-math reordering.** With `#pragma GCC optimize("fast-math")` floating-point associativity
  relaxes; reconstruction residuals may reach $10^{-5}$ rather than $10^{-7}$ for 32-bit floats.

## Variants & Generalizations

- **Daubechies-$N$ family**: increasing tap count adds vanishing moments, improving energy
  compaction for smooth signals at the cost of longer filters and larger boundary effects.
- **Biorthogonal wavelets** (e.g. CDF 9/7 used in JPEG 2000): analysis and synthesis filters
  differ but still give perfect reconstruction; not orthogonal, so Parseval does not hold.
- **Wavelet packet transform**: decomposes both approximation and detail sub-bands at every level,
  forming a full binary tree of sub-bands for adaptive best-basis selection.
- **Undecimated (stationary) DWT**: omits downsampling; translation-invariant but $O(N \log N)$.

## Applications

- **Denoising**: threshold small detail coefficients (hard or soft thresholding); edges and
  transients survive while broadband noise is suppressed.
- **Compression**: retain the few large coefficients; the rest encode to zero-runs.
- **Feature extraction for edge-ML**: wavelet energy per sub-band is a compact descriptor for
  vibration, ECG, or acoustic signals.
- **Multi-rate filtering**: each level is a critically sampled octave-band filter, useful for
  hearing-aid and audio-codec pipelines.

## Connections to Other Algorithms

- **ConvolutionCorrelation**: the inner analysis/synthesis loops are FIR convolutions; the same
  periodic-indexing trick is used.
- **FastFourierTransform / RealFastFourierTransform**: alternative time-frequency view; the DWT
  trades uniform frequency resolution for logarithmic-scale (dyadic) resolution.
- **PowerDensitySpectrum**: Welch's method is an FFT-based power estimator; the DWT gives a
  non-uniform octave-band equivalent.
- **DiscreteCosineTransform**: a block transform (fixed basis); the DWT is a multi-scale
  transform (variable support).

## References & Further Reading

- S. Mallat, "A Theory for Multiresolution Signal Decomposition: The Wavelet Representation,"
  *IEEE Trans. Pattern Analysis and Machine Intelligence*, 11(7), 674–693, 1989.
- I. Daubechies, *Ten Lectures on Wavelets*, SIAM, 1992.
- G. Strang and T. Nguyen, *Wavelets and Filter Banks*, Wellesley-Cambridge Press, 1996.
- A. V. Oppenheim and R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Ch. 11, 2010.

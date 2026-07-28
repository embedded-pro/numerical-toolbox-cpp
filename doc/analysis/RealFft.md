# Real-Input FFT (RFFT)

## Overview & Motivation

Most sensor data is real-valued: audio samples, vibration measurements, ADC readings.
A general complex FFT applied to such data wastes half its work because the imaginary
input channel is identically zero. The real-input FFT exploits this structure to deliver
roughly twice the throughput and half the memory of a complex FFT at the same transform
length. On bandwidth- and RAM-limited embedded targets this difference is significant.

## Mathematical Theory

### Prerequisites

The RFFT is a specialisation of the length-$N$ Discrete Fourier Transform

$$X[k] = \sum_{n=0}^{N-1} x[n]\, W_N^{nk}, \quad W_N = e^{-j2\pi/N}$$

for the case where every $x[n]$ is real-valued. Because $x[n] \in \mathbb{R}$, the
spectrum satisfies Hermitian symmetry: $X[N-k] = X^*[k]$. Consequently only the
$N/2 + 1$ bins $k = 0, \dots, N/2$ carry independent information.

### Decimation-in-time split

Split $x$ into its even- and odd-indexed subsequences:

$$x_e[n] = x[2n], \qquad x_o[n] = x[2n+1], \qquad n = 0, \dots, N/2 - 1.$$

Their length-$(N/2)$ DFTs are $E[k]$ and $O[k]$. The full transform decomposes as

$$X[k] = E[k] + W_N^k \, O[k], \qquad k = 0, \dots, N/2 - 1,$$

$$X[N/2 + k] = E[k] - W_N^k \, O[k], \qquad k = 0, \dots, N/2 - 1.$$

The twiddle factor $W_N^k = e^{-j2\pi k/N}$ is precomputed for $k = 1, \dots, N/2 - 1$.
The boundary cases are purely real:

$$X[0] = E[0] + O[0], \qquad X[N/2] = E[0] - O[0].$$

### Inverse transform

Given the $N/2 + 1$ unique bins, the even and odd sub-spectra are recovered using the
inverse of the split relations. For $k = 1, \dots, N/2 - 1$:

$$E[k] = \tfrac{1}{2}(X[k] + X^*[N/2 - k]),$$

$$O[k] = W_N^{-k} \cdot \tfrac{1}{2}(X[k] - X^*[N/2 - k]).$$

Two independent length-$(N/2)$ inverse DFTs then reconstruct $x_e$ and $x_o$, which
interleave back to $x$.

## Complexity Analysis

| Case    | Time              | Space                  | Notes                                         |
|---------|-------------------|------------------------|-----------------------------------------------|
| Forward | $O(N \log N / 2)$ | $O(N)$ stack (bounded) | Two half-length DFTs plus $O(N)$ combine pass |
| Inverse | $O(N \log N / 2)$ | $O(N)$ stack (bounded) | Two half-length IDFTs plus interleave pass    |

The constant factor is approximately half that of a complex $N$-point FFT because
both sub-DFTs exploit real-input symmetry internally (they receive real-only vectors).

## Step-by-Step Walkthrough

Input: $x = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]$ (unit impulse, $N = 16$).

1. **Split**: $x_e = [1, 0, 0, 0, 0, 0, 0, 0]$, $x_o = [0, 0, 0, 0, 0, 0, 0, 0]$.
2. **Sub-DFTs**: $E[k] = 1$ for all $k$; $O[k] = 0$ for all $k$.
3. **Combine**: $X[k] = E[k] + W_{16}^k \cdot 0 = 1$ for $k = 0, \dots, 8$.
4. **Result**: $|X[k]| = 1$ — flat magnitude, the hallmark of a unit impulse.

## Pitfalls & Edge Cases

- **N must be an even power of two.** The underlying radix-2 engine imposes this; a
  `static_assert` enforces it at compile time.
- **DC and Nyquist bins are exactly real.** Floating-point errors in the sub-DFTs may
  introduce tiny imaginary residuals; downstream code should not rely on them being
  precisely zero.
- **Round-trip accumulation.** Two forward and two inverse sub-DFTs compound rounding
  errors. The round-trip error is $O(\varepsilon \log N)$ where $\varepsilon$ is the
  floating-point machine epsilon.
- **Spectrum aliasing at Nyquist.** The bin at $k = N/2$ is real and unique only when
  $N$ is even, which is always the case here.

## Variants & Generalizations

- **Split-radix RFFT**: replaces the two half-length DFTs with a mixed radix-2/4
  decomposition, reducing the arithmetic count by roughly 30% versus the radix-2 split
  used here.
- **In-place RFFT**: packs the $N/2 + 1$ complex output into the same $N$-sample real
  buffer, halving memory further.
- **Zero-padded RFFT**: appending zeros to $x$ before the transform gives finer
  frequency resolution at the cost of more compute.

## Applications

- Spectrum analysis of real sensor signals (vibration, audio, pressure).
- Cross-correlation and convolution of real sequences via `ConvolutionCorrelation`.
- Power spectral density estimation via `PowerDensitySpectrum`.
- Feature extraction in embedded machine learning pipelines.

## Connections to Other Algorithms

- **FastFourierTransform / FastFourierTransformRadix2Impl**: the injected complex engine
  that does the heavy butterfly work; RFFT adds only a linear-time split pass on top.
- **PowerDensitySpectrum**: consumes the output of the RFFT to estimate spectral power.
- **ConvolutionCorrelation**: uses the FFT for $O(N \log N)$ convolution; RFFT halves
  the cost when both operands are real.
- **HilbertTransform**: also builds on the FFT engine with a frequency-domain
  manipulation pass, similar in structure to the RFFT split step.

## References & Further Reading

- H. V. Sorensen, D. L. Jones, M. T. Heideman, C. S. Burrus, "Real-valued fast Fourier
  transform algorithms," *IEEE Trans. Acoustics, Speech, and Signal Processing*,
  35(6), 849–863, 1987.
- A. V. Oppenheim, R. W. Schafer, *Discrete-Time Signal Processing*, 3rd ed., Ch. 9.
- S. W. Smith, *The Scientist and Engineer's Guide to Digital Signal Processing*,
  Ch. 12 (available at dspguide.com).

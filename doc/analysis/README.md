# Analysis

Signal analysis algorithms for frequency-domain decomposition and spectral estimation.

## Algorithms

| Algorithm                                               | Description                                                                                      |
|---------------------------------------------------------|--------------------------------------------------------------------------------------------------|
| [Fast Fourier Transform](FastFourierTransform.md)       | Efficient computation of the Discrete Fourier Transform using the Cooley-Tukey radix-2 algorithm |
| [Real-Input FFT](RealFastFourierTransform.md)                            | Length-N real FFT via even/odd split into two N/2-point complex DFTs, halving compute and memory |
| [Power Spectral Density](PowerDensitySpectrum.md)       | Estimation of signal power distribution across frequencies using Welch's method                  |
| [Discrete Cosine Transform](DiscreteCosineTransform.md) | Real-valued frequency decomposition via cosine basis functions, computed through FFT             |
| [Signal Detectors](SignalDetectors.md)                  | Peak hold, zero-crossing counter, and RMS envelope detectors for real-time signal monitoring     |
| [Decibels](Decibels.md)                                 | `ToDecibels` / `FromDecibels` conversion helpers with zero-floor guard, plus attenuation and ripple utilities |
| [Goertzel Algorithm](GoertzelAlgorithm.md)              | Single-bin DFT via a second-order recurrence for O(N) tone detection with O(1) memory                         |
| [Hilbert Transform](HilbertTransform.md)                | Analytic signal and instantaneous amplitude/phase/frequency via FFT one-sided spectrum or FIR approximation   |

## Sub-domains

| Sub-domain                          | Description                                                                      |
|-------------------------------------|----------------------------------------------------------------------------------|
| [Windowing](../windowing/README.md) | Window functions (Rectangular, Hamming, Hanning, Blackman) for FFT preprocessing |

> **Note:** For control system analysis tools (Frequency Response, Root Locus), see [Control Analysis](../control_analysis/README.md).

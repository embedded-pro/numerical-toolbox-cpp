# Analysis

Signal analysis algorithms for frequency-domain decomposition and spectral estimation.

## Algorithms

| Algorithm                                               | Description                                                                                      |
|---------------------------------------------------------|--------------------------------------------------------------------------------------------------|
| [Fast Fourier Transform](FastFourierTransform.md)       | Efficient computation of the Discrete Fourier Transform using the Cooley-Tukey radix-2 algorithm |
| [Power Spectral Density](PowerDensitySpectrum.md)       | Estimation of signal power distribution across frequencies using Welch's method                  |
| [Discrete Cosine Transform](DiscreteCosineTransform.md) | Real-valued frequency decomposition via cosine basis functions, computed through FFT             |

## Sub-domains

| Sub-domain                          | Description                                                                      |
|-------------------------------------|----------------------------------------------------------------------------------|
| [Windowing](../windowing/README.md) | Window functions (Rectangular, Hamming, Hanning, Blackman) for FFT preprocessing |

> **Note:** For control system analysis tools (Frequency Response, Root Locus), see [Control Analysis](../control_analysis/README.md).

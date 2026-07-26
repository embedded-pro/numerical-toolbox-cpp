# Real-Input FFT (RFFT) — Overview

## What it is
A fast Fourier transform specialised for **real** signals. Because ADC samples have no imaginary
part, half of a general complex FFT is redundant. The RFFT packs the real data into a half-length
complex array, runs one `N/2`-point complex FFT, and untangles the result into the `N/2+1` unique
spectral bins.

## Why it matters (embedded)
Real sensor data is the common case. The pack trick gives roughly **2× throughput and half the
memory** of a complex FFT at the same length — a big deal when both cycles and RAM are scarce. It
reuses the library's existing radix-2 engine rather than adding a second FFT implementation.

## How it works (intuition)
Two real sequences can ride in the real and imaginary channels of one complex FFT. Feed the even
samples as the real part and the odd samples as the imaginary part; a single `N/2` FFT then carries
enough information to recover the full spectrum. A short "split" pass uses conjugate symmetry and a
twiddle factor to separate the two interleaved transforms and recombine them into `X[k]`. The inverse
simply runs the steps backwards, exploiting Hermitian symmetry `X[N−k] = conj(X[k])`.

## Key parameters
- **N (transform length)** — even power of two; the underlying complex FFT is size `N/2`.
- **injected FFT engine** — the existing `FastFourierTransform`, supplied by dependency injection.
- **output length** — only bins `0 … N/2` are unique and returned.

## Reference
H. V. Sorensen, D. L. Jones, M. T. Heideman, C. S. Burrus, "Real-valued fast Fourier transform
algorithms," *IEEE Trans. Acoustics, Speech, and Signal Processing*, 35(6), 849–863, 1987.

## See also
`FastFourierTransform` (the injected complex engine), `PowerDensitySpectrum`, `HilbertTransform`
and `ConvolutionCorrelation` (both build on the FFT).

# Discrete Wavelet Transform (Haar / Daubechies) — Overview

## What it is
A multiresolution decomposition that splits a signal into **approximation** (coarse, low-frequency)
and **detail** (fine, high-frequency) coefficients at several scales. Unlike the FFT — which uses
infinitely-long sinusoids — wavelets are localised in **both time and frequency**, so they capture
*when* an event happens as well as *what* frequencies it contains.

## Why it matters (embedded)
Wavelets are the workhorse for **denoising, compression, and time-frequency feature extraction** in
condition monitoring and edge-ML pipelines. Thresholding the detail coefficients removes noise while
preserving edges/transients that a linear filter would smear. The transform is O(N) and streams
through short FIR filters — friendly to constrained hardware.

## How it works (intuition)
One stage passes the signal through a matched pair of **quadrature-mirror filters** — a low-pass
(scaling) and a high-pass (wavelet) — then downsamples each by two. The low-pass output (the
approximation) is fed back into the same stage, building a dyadic tree of ever-coarser scales. The
inverse upsamples and runs the synthesis pair; because the filters are orthogonal, the original
signal is reconstructed perfectly. **Haar** (2 taps) is the simplest wavelet; **Daubechies** wavelets
add taps for smoother shapes with more vanishing moments.

## Key parameters
- **wavelet family / Taps** — Haar (2), db2 (4), db4 (8)… more taps = smoother, more vanishing moments.
- **Levels** — decomposition depth; `N` must be divisible by `2^Levels`.
- **boundary handling** — periodic vs symmetric extension (must match on analysis and synthesis).

## Reference
S. Mallat, "A Theory for Multiresolution Signal Decomposition: The Wavelet Representation," *IEEE
Trans. Pattern Analysis and Machine Intelligence*, 11(7), 674–693, 1989; I. Daubechies, *Ten Lectures
on Wavelets*, SIAM, 1992.

## See also
`ConvolutionCorrelation` (the FIR filtering underneath), `FastFourierTransform` / `RealFastFourierTransform`
(alternative time-frequency view), `math::RecursiveBuffer`.

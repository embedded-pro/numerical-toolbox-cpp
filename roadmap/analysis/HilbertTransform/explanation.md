# Hilbert Transform / Analytic Signal — Overview

## What it is
The Hilbert transform applies a **−90° phase shift** to every frequency component of a real signal.
Pairing the original signal (real part) with its Hilbert transform (imaginary part) forms the
**analytic signal** — a complex signal with no negative-frequency content whose magnitude and angle
give the instantaneous **amplitude (envelope)**, **phase**, and **frequency**.

## Why it matters (embedded)
It is the standard tool for AM demodulation, envelope detection, and — crucially — **vibration and
bearing analysis** in condition monitoring, where the envelope of a high-frequency resonance reveals
low-frequency fault signatures. It is also used for single-sideband modulation and
instantaneous-frequency tracking.

## How it works (intuition)
Two equivalent routes:
- **FFT method (Marple):** transform the block, keep DC/Nyquist, **double the positive frequencies,
  zero the negatives**, then inverse-transform. The result is exactly analytic — best for block
  processing.
- **FIR method:** a linear-phase antisymmetric filter whose taps approximate `2/(πk)` performs the
  90° shift sample-by-sample, paired with a matched delay of the original — best for streaming.

Once you have the analytic signal, the envelope is its magnitude, the phase is `atan2(Im, Re)`, and
the instantaneous frequency is the (unwrapped) time-derivative of that phase.

## Key parameters
- **N (block length)** — FFT-method resolution and latency.
- **Taps** — FIR length; more taps approach an ideal shift but add fixed latency `(Taps−1)/2`.
- **sample time Ts** — converts phase increments into frequency.

## Reference
S. L. Marple, "Computing the Discrete-Time Analytic Signal via FFT," *IEEE Trans. Signal Processing*,
47(9), 2600–2603, 1999.

## See also
`RealFastFourierTransform` / `FastFourierTransform` (FFT back-end), `ConvolutionCorrelation` (FIR realisation),
`SignalDetectors` (cheap non-coherent envelope).

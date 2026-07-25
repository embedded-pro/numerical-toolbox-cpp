# Goertzel Algorithm — Overview

## What it is
A way to compute **one** DFT bin `X[k]` with a tiny second-order IIR recurrence instead of a full
FFT. It streams samples in one at a time and produces the bin's real/imaginary value (or magnitude)
after exactly `N` samples.

## Why it matters (embedded)
When you only care about a few known frequencies — DTMF touch-tones, a pilot/sync tone, mains-hum
monitoring — Goertzel is dramatically cheaper than an FFT: **O(1) memory** (two state words, no input
buffer, no twiddle table) and one multiply-add per sample. Ideal for the smallest MCUs.

## How it works (intuition)
The DFT sum is reorganised into a resonator tuned to bin `k`. Each sample drives the recurrence
`s[n] = x[n] + 2cos(2πk/N)·s[n−1] − s[n−2]`; the resonator "rings" in proportion to how much energy
the input carries at that frequency. After `N` samples, two cheap combinations of the final states
give the exact DFT coefficient. Because the pole sits on the unit circle it acts as a perfect
(marginally stable) oscillator over one finite block.

## Key parameters
- **k (target bin)** — which DFT frequency, `f = k·fs/N`; choose `N` so your tone lands near integer `k`.
- **N (block length)** — sets frequency resolution `fs/N` and detection latency.
- **coeff = 2·cos(2πk/N)** — the single precomputed constant that tunes the resonator.

## Reference
G. Goertzel, "An Algorithm for the Evaluation of Finite Trigonometric Series," *American
Mathematical Monthly*, 65(1), 34–35, 1958.

## See also
`FastFourierTransform` / `RealFft` (all bins at once), `SignalDetectors` (cheap non-selective
detection), `math::Complex`.

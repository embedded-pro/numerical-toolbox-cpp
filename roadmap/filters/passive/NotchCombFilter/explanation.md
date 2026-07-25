# Notch / Comb Filter — Overview

## What it is
Two related narrow-band rejecters: a **notch** removes a single frequency (a biquad with its zeros
on the unit circle), and a **comb** removes a fundamental *and all its harmonics* using a single
delayed feedforward/feedback path.

## Why it matters (embedded)
The dominant interference in bio-signals (ECG/EEG/EMG) and precision instrumentation is `50/60 Hz`
mains hum and its harmonics. A notch surgically deletes the fundamental while leaving the rest of
the spectrum intact; a comb cleans up the whole harmonic series at once for the price of one delay
line.

## How it works (intuition)
A notch places a pair of zeros exactly on the unit circle at the offending frequency and nearby
poles just inside to keep the rest of the response flat — `Q` controls how tight the notch is. A
comb delays the signal by `D` samples and subtracts (or adds) it; the delay creates evenly spaced
nulls at every multiple of `fs/D`, which lines up perfectly with a harmonic series.

## Key parameters
- **f0, Q (notch)** — centre frequency and width; higher `Q` = narrower, sharper notch.
- **D, gain, feedback (comb)** — delay length sets null spacing (`fs/D`), gain sets depth, feedback
  mode trades sharper nulls for a stability constraint (`gain < 1`).

## Reference
R. Bristow-Johnson, "Cookbook formulae for audio EQ biquad filter coefficients" (notch);
R. G. Lyons, *Understanding Digital Signal Processing* (comb filters).

## See also
`BiquadCascade` (general biquad realization), `IirFilterDesign` (band-stop design),
`GoertzelAlgorithm` (detect the tone a notch removes).

# Passive Filters

Classical signal-processing filters implemented as second-order sections, FIR taps, or recursive accumulators — no dynamic model, no feedback from an observer.

## Algorithms

| Algorithm                                                    | Description                                                                                               |
|--------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| [Biquad / SOS Cascade](BiquadCascade.md)                     | Runtime cascade of second-order IIR sections; the building block for all SOS-based filters                |
| [CIC Filter](CicFilter.md)                                   | Cascaded Integrator-Comb decimation/interpolation filter with no multipliers                              |
| [Exponential Moving Average](ExponentialMovingAverage.md)    | Single-pole recursive smoother — the cheapest low-pass filter                                             |
| [FIR](Fir.md)                                                | Finite Impulse Response filter with arbitrary tap coefficients                                            |
| [IIR](Iir.md)                                                | Direct-Form II biquad IIR filter with user-supplied coefficients                                          |
| [IIR Filter Design](IirFilterDesign.md)                      | On-device Butterworth / Chebyshev-I design via bilinear transform; emits SOS coefficients for BiquadCascade |
| [Median Filter](MedianFilter.md)                             | Non-linear rank-based filter for impulsive-noise rejection                                                |
| [Moving Average](MovingAverage.md)                           | Length-N boxcar filter via incremental running sum                                                        |
| [Notch / Comb Filter](NotchCombFilter.md)                    | Narrow band-stop notch and periodic-harmonic comb filter                                                  |
| [Savitzky-Golay Filter](SavitzkyGolayFilter.md)              | Polynomial least-squares smoothing preserving peak shapes and derivatives                                 |

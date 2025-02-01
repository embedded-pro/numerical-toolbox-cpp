# Fast Fourier Transform (FFT) Implementation Guide

## Mathematical Background

### The Continuous Fourier Transform

The Fourier Transform is a mathematical tool that decomposes a continuous function into its constituent frequencies. For a continuous-time signal x(t), its Fourier Transform X(f) is:

```
X(f) = ∫[from -∞ to +∞] x(t) * e^(-2πift) dt
```

And its inverse transform is:

```
x(t) = ∫[from -∞ to +∞] X(f) * e^(2πift) df
```

The transform effectively converts a signal from the time domain to the frequency domain, revealing its frequency components. This has profound implications in:

1. **Physics**: Analyzing wave phenomena, quantum mechanics
2. **Signal Processing**: Understanding signal composition
3. **Differential Equations**: Solving complex equations by transforming them to algebraic ones
4. **Image Processing**: Analyzing spatial frequencies

The key insight of Fourier analysis is that any continuous, periodic function can be represented as a sum of sinusoidal functions of different frequencies. Even non-periodic functions can be represented using the continuous spectrum of frequencies through the Fourier Transform.

### From Continuous to Discrete

When working with digital systems, we need to:
1. Sample the continuous signal at discrete time intervals
2. Process a finite number of samples

This leads us to the Discrete Fourier Transform (DFT), which is a sampled version of the continuous Fourier Transform. The relationship between sampling in time domain and frequency domain is governed by the:
- Nyquist-Shannon sampling theorem: To accurately represent a signal, sample at least twice the highest frequency component
- Aliasing: When sampling below the Nyquist rate, high frequencies appear as false low frequencies

### The Discrete Fourier Transform (DFT)

The Discrete Fourier Transform converts a sequence of N complex numbers {xn} into another sequence of complex numbers {Xk}:

```
Xk = Σ(n=0 to N-1) xn * e^(-2πi*k*n/N)
```

where:
- n is the time index
- k is the frequency index
- N is the number of samples
- i is the imaginary unit

### The Fast Fourier Transform Algorithm

The FFT is an optimized algorithm to compute the DFT. While the direct DFT computation requires O(N²) operations, the FFT reduces this to O(N log N) by:

1. Exploiting the periodicity of e^(-2πi)
2. Decomposing the DFT into smaller DFTs (divide-and-conquer)
3. Using the symmetry properties of the complex exponential

Our implementation uses the Radix-2 Cooley-Tukey algorithm, which requires the input length to be a power of 2.

## Implementation Details

### Class Structure

```cpp
template<typename QNumberType>
class FastFourierTransform {
    virtual VectorComplex& Forward(VectorReal& input) = 0;
    virtual VectorReal& Inverse(VectorComplex& input) = 0;
};
```

The implementation is split into:
1. Base abstract class (`FastFourierTransform`)
2. Radix-2 implementation (`FastFourierTransformRadix2Impl`)
3. Twiddle factors helper class (`TwiddleFactors`)

### Key Components

1. **Twiddle Factors**: Pre-computed complex exponentials (e^(-2πi*k/N)) used in the butterfly operations
2. **Bit Reversal**: Reorders input data to optimize butterfly operations
3. **Butterfly Operations**: The core computation units that combine results from smaller DFTs

## Usage Guide

### Basic Usage

```cpp
// Create an FFT instance (N must be a power of 2)
constexpr std::size_t N = 1024;
TwiddleFactorsImpl<float, N/2> twiddleFactors;
FastFourierTransformRadix2Impl<float, N> fft(twiddleFactors);

// Prepare input data
infra::BoundedVector<float>::WithMaxSize<N> input;
for (const auto& value : signal)
    input.push_back(value);

// Perform forward transform
auto& frequencyDomain = fft.Forward(input);

// Perform inverse transform if needed
auto& reconstructedSignal = fft.Inverse(frequencyDomain);
```

### Computing Magnitude Spectrum

To compute the magnitude spectrum from the complex FFT output:

```cpp
std::vector<float> magnitudes(N/2);
for (std::size_t i = 0; i < N/2; ++i) {
    auto complex = frequencyDomain[i];
    magnitudes[i] = std::sqrt(
        complex.Real() * complex.Real() + 
        complex.Imaginary() * complex.Imaginary()
    );
}
```

### Frequency Resolution

The frequency resolution of the FFT is:
```
Δf = sampleRate / N
```

To convert bin index to frequency:
```cpp
float frequency = binIndex * sampleRate / N;
```

## Best Practices

1. **Input Length**: Always use a power of 2 for the input length (e.g., 256, 512, 1024, 2048)
2. **Memory Management**: The implementation uses `BoundedVector` to prevent heap allocations
3. **Numerical Precision**: Template parameter `QNumberType` allows choosing between float/double precision
4. **Scaling**: Remember that the inverse FFT output needs to be scaled by 1/N

## Common Applications

1. **Spectral Analysis**: Analyzing frequency content of signals
2. **Filtering**: Multiplication in frequency domain for efficient filtering
3. **Feature Extraction**: Extracting frequency-domain features for classification
4. **Data Compression**: Removing high-frequency components

## Example: Signal Analysis

```cpp
// Generate a test signal with multiple frequency components
auto signal = SignalGenerator<float>::GenerateSignal(N, sampleRate);

// Perform FFT
auto& frequencyDomain = fft.Forward(input);

// Analyze specific frequency components
for (std::size_t i = 0; i < N/2; ++i) {
    float frequency = i * sampleRate / N;
    float magnitude = std::sqrt(
        frequencyDomain[i].Real() * frequencyDomain[i].Real() +
        frequencyDomain[i].Imaginary() * frequencyDomain[i].Imaginary()
    );
    // Process frequency components...
}
```

## Performance Considerations

1. **Twiddle Factors**: Pre-computed to avoid repeated calculation
2. **Bit Reversal**: Optimized using lookup tables for small N
3. **Memory Access**: In-place computation to minimize memory usage
4. **Template Metaprogramming**: Used to optimize compile-time operations

## Limitations and Future Improvements

1. Currently only supports power-of-2 lengths (Radix-2)
2. Could be extended to support:
   - Mixed-radix FFT
   - Real-valued FFT optimization
   - Parallel computation
   - Windowing functions

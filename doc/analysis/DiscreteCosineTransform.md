# Discrete Cosine Transform (DCT) Implementation Guide

## Process Overview

The DCT transforms a sequence of data points into a sum of cosine functions oscillating at different frequencies. Our implementation specifically focuses on DCT-II, which is the most commonly used variant, especially in signal compression applications.

The following diagram illustrates the key steps in our DCT implementation:

![DCT Processing Flow](doc/analysis/DctFlow.svg)

The above diagram shows the main processing stages:
1. Input signal preprocessing (reordering and scaling)
2. FFT computation on preprocessed data
3. Post-processing to obtain DCT coefficients

The DCT decomposes signals into different frequency components using cosine basis functions:

![DCT Basis Functions](doc/analysis/DctBasis.svg)

The diagram shows how different frequency components (k=0,1,2,3) contribute to the final signal representation. The k=0 component represents the DC (average) value, while higher k values represent increasingly higher frequency oscillations.

The key processing steps are:
1. Input signal preprocessing
2. FFT computation
3. Post-processing to obtain DCT coefficients
4. (For inverse) Reverse process to recover the original signal

## Mathematical Background

### Discrete Cosine Transform

The DCT-II of a sequence x[n] of length N is defined as:

```
X[k] = Σ(n=0 to N-1) x[n] * cos(π/N * (n + 1/2) * k)
```

where:
- x[n] is the input sequence
- X[k] are the DCT coefficients
- N is the sequence length
- k is the frequency index

### FFT-Based Implementation

Our implementation uses FFT to compute DCT efficiently. The process involves:

1. Rearranging the input sequence
2. Computing FFT
3. Multiplying with twiddle factors

The mathematical relationship is:
```
DCT[k] = 2 * Real{W[k] * F[k]}
```
where:
- F[k] is the FFT of the rearranged sequence
- W[k] = exp(-jπk/2N) is the twiddle factor
- Real{} denotes the real part

## Implementation Details

### Class Structure

```cpp
template<typename QNumberType, std::size_t Length>
class DiscreteConsineTransform {
public:
    explicit DiscreteConsineTransform(
        FastFourierTransform<QNumberType>& fft);
        
    VectorReal& Forward(VectorReal& input);
    VectorReal& Inverse(VectorReal& input);
};
```

### Key Components

1. **Input Processing**:
   - Reorders input for FFT compatibility
   - Ensures fixed-point range compliance
   - Handles boundary conditions

2. **FFT Interface**:
   - Uses templated FFT implementation
   - Manages complex intermediate results
   - Optimizes memory usage

3. **Scaling Management**:
   - Maintains fixed-point range [-1, 1)
   - Applies appropriate scaling factors
   - Handles normalization

4. **Fixed-Point Considerations**:
   - Pre-computed trigonometric tables
   - Optimized multiplication sequences
   - Range-checked operations

## Usage Guide

### Basic Usage

```cpp
// Define system parameters
constexpr std::size_t N = 1024;          // Transform size
using NumberType = math::Q15;             // Fixed-point type

// Create FFT instance
TwiddleFactors<NumberType, N/2> twiddleFactors;
FastFourierTransformRadix2Impl<NumberType, N> fft(twiddleFactors);

// Create DCT analyzer
DiscreteConsineTransform<NumberType, N> dct(fft);

// Prepare input data
infra::BoundedVector<NumberType>::WithMaxSize<N> signal;
// ... fill signal with data ...

// Calculate DCT
auto& coefficients = dct.Forward(signal);

// Process coefficients
for (std::size_t i = 0; i < N; ++i) {
    // Process DCT coefficients...
}

// Inverse transform if needed
auto& reconstructed = dct.Inverse(coefficients);
```

### Example: Image Compression

```cpp
// Create DCT processor for 8x8 blocks
constexpr std::size_t blockSize = 8;
DiscreteConsineTransform<float, blockSize> dct(fft);

// Process image block
BoundedVector<float>::WithMaxSize<blockSize> block;
// ... fill block with pixel data ...

// Transform
auto& dctCoeffs = dct.Forward(block);

// Quantize coefficients (compression)
for (auto& coeff : dctCoeffs) {
    coeff = Quantize(coeff, quantizationMatrix[i]);
}

// Inverse transform
auto& reconstructed = dct.Inverse(dctCoeffs);
```

## Best Practices

1. **Input Preparation**:
   - Ensure data is within fixed-point range
   - Pre-scale values if necessary
   - Consider block size requirements

2. **Type Selection**:
   - Use Q15/Q31 for embedded systems
   - Float for desktop applications
   - Consider precision requirements

3. **Performance Optimization**:
   - Use power-of-2 lengths
   - Reuse transform instances
   - Leverage pre-computed tables

4. **Memory Management**:
   - Use BoundedVector for static allocation
   - Minimize temporary buffers
   - Consider alignment requirements

## Common Applications

1. **Data Compression**:
   - Image compression (JPEG)
   - Audio compression
   - Video encoding

2. **Signal Processing**:
   - Feature extraction
   - Pattern recognition
   - Spectral analysis

3. **Scientific Computing**:
   - Numerical solutions
   - Fast convolution
   - Data reduction

## Performance Considerations

1. **Compile-Time Optimization**:
   - Fixed sizes known at compile time
   - Template specialization
   - Inline expansion

2. **Memory Efficiency**:
   - Static allocation
   - Minimal working buffers
   - In-place operations where possible

3. **Computational Efficiency**:
   - FFT-based implementation
   - Pre-computed tables
   - Optimized fixed-point math

## Limitations and Future Improvements

1. Current limitations:
   - Power-of-2 size requirement
   - Real-valued input only
   - Single precision support
   - Basic DCT-II variant only

2. Possible extensions:
   - Other DCT variants (I, III, IV)
   - Arbitrary size support
   - Multi-dimensional transform
   - Parallel processing
   - SIMD optimization
   - Streaming interface
   - Block processing

## Error Handling

1. Static assertions verify:
   - Power-of-2 length
   - Valid numeric types
   - Size constraints
   - FFT compatibility

2. Runtime checks:
   - Input range validation
   - Buffer management
   - Overflow prevention
   - Error propagation

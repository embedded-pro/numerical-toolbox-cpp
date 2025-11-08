# GitHub Copilot Instructions for Numerical Toolbox

## Project Overview

This is a numerical algorithms library providing digital signal processing (DSP), control algorithms, filters, optimizers, and estimators for embedded systems. The library is designed with strict performance and memory constraints, targeting resource-constrained microcontrollers while maintaining numerical accuracy and real-time determinism.

## Repository Structure

- **numerical/**: Core implementation of algorithms organized by domain:
  - `analysis/`: Signal analysis (FFT, PSD, DCT)
  - `controllers/`: Control algorithms (PID controllers)
  - `estimators/`: Estimation algorithms (Linear Regression, Yule-Walker)
  - `filters/`: Digital filters (FIR, IIR, Kalman)
  - `math/`: Mathematical foundations (QNumber, Matrix, Complex, Trigonometric functions)
  - `neural_network/`: Neural network components (layers, activations, optimizers)
  - `solvers/`: Numerical solvers (Levinson-Durbin)
  - `windowing/`: Window functions for signal processing
- **examples/**: Demonstrates library usage with practical implementations
- **doc/**: Comprehensive documentation for each algorithm with mathematical background
- **embedded-infra-lib/**: Submodule containing infrastructure foundations and utilities for embedded systems

## Critical Constraints

### Memory Management

- **NO HEAP ALLOCATION**: Avoid `new`, `delete`, `malloc`, `free`, and `std::make_unique/std::make_shared` at all costs
- **NO DYNAMIC CONTAINERS**: Replace standard library containers that use dynamic allocation:
  - Use `infra::BoundedVector` instead of `std::vector`
  - Use `infra::BoundedString` instead of `std::string`
  - Use `infra::BoundedDeque` instead of `std::deque`
  - Use `infra::BoundedList` instead of `std::list`
- **STATIC ALLOCATION**: All memory must be allocated at compile-time or on the stack
- **AVOID RECURSION**: Stack usage must be predictable and minimal

### Performance Requirements

- **REAL-TIME CONSTRAINTS**: Code must execute deterministically within strict timing requirements
- **MINIMIZE BRANCHING**: Reduce conditional statements in critical paths (e.g., FOC control loops)
- **AVOID VIRTUAL CALLS IN ISR**: Virtual function calls add overhead; avoid in interrupt service routines
- **INLINE CRITICAL CODE**: Use `inline` for small, frequently-called functions
- **CONST CORRECTNESS**: Mark all non-mutating methods as `const` for compiler optimization
- **PREFER CONSTEXPR**: Use `constexpr` for compile-time calculations when possible

### Memory Consumption

- **MINIMIZE FOOTPRINT**: Every byte counts in embedded systems
- **USE FIXED-SIZE TYPES**: Prefer `uint8_t`, `int32_t`, etc., over `int` for predictable sizing
- **PACK STRUCTURES**: Use pragma pack or compiler attributes when appropriate
- **AVOID COPYING**: Use references and move semantics to prevent unnecessary copies
- **MEASURE SIZE**: Be aware of sizeof() for all data structures

## Code Organization Guidelines

### Core Implementation (`numerical/`)

Place code here when it is:
- Generic mathematical and numerical algorithms
- Signal processing and analysis functions
- Control algorithms (PID, state-space controllers)
- Digital filters (FIR, IIR, Kalman)
- Estimators and solvers
- Neural network components
- Hardware-agnostic and reusable across projects
- Organized by domain (analysis, controllers, filters, estimators, math, neural_network, solvers, windowing)

### Examples (`examples/`)

Place code here when it is:
- Demonstrating proper usage of library components
- Showing practical implementations and patterns
- Providing working reference implementations
- Testing algorithms in realistic scenarios
- Each example should be self-contained and demonstrate specific features

### Documentation (`doc/`)

When implementing or modifying algorithms:
- **ALWAYS UPDATE DOCUMENTATION**: Each algorithm must have corresponding documentation in `doc/`
- Include mathematical background and theory
- Provide implementation details and considerations
- Add usage examples and expected behaviors
- Document numerical properties and limitations
- Keep documentation in sync with code changes

## Coding Style and Patterns

### Dependency Injection

- Use constructor injection for dependencies
- Pass interfaces, not concrete implementations when appropriate
- Example:
  ```cpp
  namespace controllers
  {
      class PidController
      {
      public:
          PidController(PidTunings tunings, PidLimits limits)
              : tunings(tunings), limits(limits) {}
      private:
          PidTunings tunings;
          PidLimits limits;
      };
  }
  ```

### Error Handling

- Use `infra::Optional` for functions that may not return a value
- Return error codes or status enums, not exceptions
- Assert preconditions in debug builds with `assert()` or `really_assert()`

### Testing

- Write unit tests using GoogleTest
- Test with multiple numeric types (float, Q15, Q31) using typed tests
- Mock dependencies for isolation
- Aim for high code coverage, especially in numerical algorithms
- Verify numerical accuracy and stability
- Test edge cases and boundary conditions
- Example typed test pattern:
  ```cpp
  template<typename T>
  class TestAlgorithm : public testing::Test {};
  
  using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
  TYPED_TEST_SUITE(TestAlgorithm, TestedTypes);
  
  TYPED_TEST(TestAlgorithm, computes_correct_result)
  {
      // Test with TypeParam
  }
  ```

## Common Patterns

### Instead of this (BAD):
```cpp
std::vector<float> samples;
samples.push_back(value);

std::string message = "Error: " + errorCode;

auto ptr = std::make_unique<Filter>();

// Using native types without consideration for fixed-point
float coefficient = 0.5f;
```

### Do this (GOOD):
```cpp
infra::BoundedVector<float>::WithMaxSize<100> samples;
samples.push_back(value);

infra::BoundedString::WithStorage<64> message;
message = "Error: ";

// Stack allocation
Fir<float> filter;

// Template for multiple numeric types
template<typename T>
class Algorithm
{
    T coefficient; // Works with float, Q15, Q31
};
```

## Additional Guidelines

- **RAII**: Use Resource Acquisition Is Initialization for resource management
- **INTERFACES**: Define interfaces (pure virtual classes) for testability and flexibility
- **NAMESPACE**: Use appropriate namespaces by domain:
  - `analysis::` - Signal analysis algorithms
  - `controllers::` - Control system algorithms
  - `estimators::` - Estimation algorithms
  - `filters::passive::` - Passive filters (FIR, IIR)
  - `filters::active::` - Active filters (Kalman)
  - `math::` - Mathematical utilities and functions
  - `neural_network::` - Neural network components
  - `solvers::` - Numerical solvers
  - `windowing::` - Window functions
- **SELF-DOCUMENTING CODE**: Write clear, self-explanatory code with descriptive names
- **UNITS**: Be explicit about units (radians, Hz, samples, normalized) in variable names or comments
- **NUMERICAL TYPES**: Support multiple numeric representations:
  - `float` for floating-point implementations
  - `math::Q15` for 16-bit fixed-point (Q15 format)
  - `math::Q31` for 32-bit fixed-point (Q31 format)
  - Use templates to support all three types when possible
  - Consider fixed-point for performance-critical code on microcontrollers without FPU
- **ALGORITHM PROPERTIES**: Document and preserve:
  - Numerical stability characteristics
  - Computational complexity (O-notation)
  - Memory requirements
  - Range and precision limitations
  - Real-time suitability
- **CODE FORMATTING**: Strictly adhere to the rules defined in `.clang-format` for consistent code style
- **TEMPLATE USAGE**: Use templates for type-generic algorithms while maintaining type safety
- **CONSTEXPR CALCULATIONS**: Maximize compile-time computation for lookup tables and constants

## Build System

- CMake-based build with presets for different targets
- Support for host (simulation and testing) and embedded targets
- Separate build configurations for Debug, Release, RelWithDebInfo
- GoogleTest for unit testing
- Optional build of examples via `NUMERICAL_TOOLBOX_BUILD_EXAMPLES`
- Optional compiler optimizations via `NUMERICAL_TOOLBOX_ENABLE_OPTIMIZATIONS`

## Version Control

- Keep commits atomic and focused
- Write clear commit messages
- Update CHANGELOG.md according to release-please conventions
- When modifying algorithms, update corresponding documentation in `doc/`

## Numerical Algorithm Best Practices

### Fixed-Point Arithmetic

- **Q15 Format**: 16-bit signed fixed-point with 15 fractional bits
  - Range: [-1, 0.9999694824...]
  - Use for moderate precision requirements on 16-bit microcontrollers
- **Q31 Format**: 32-bit signed fixed-point with 31 fractional bits
  - Range: [-1, 0.9999999995...]
  - Use for higher precision requirements on 32-bit microcontrollers
- **Overflow Prevention**: Scale intermediate calculations to prevent overflow
- **Saturation**: Use saturating arithmetic where appropriate
- **Conversion**: Provide explicit conversion functions between formats

### Algorithm Implementation Pattern

When implementing a new algorithm:

1. **Create interface/base class** if multiple implementations exist
2. **Implement for float first** to validate mathematical correctness
3. **Add Q15 and Q31 implementations** using the same interface
4. **Write comprehensive tests** for all numeric types
5. **Document in `doc/`** with:
   - Mathematical background
   - Implementation details
   - Usage examples
   - Numerical considerations (stability, range, precision)
6. **Create example in `examples/`** demonstrating practical usage
7. **Consider SIMD optimizations** where applicable using `math::SingleInstructionMultipleData`

### Performance Optimization Guidelines

- **Lookup Tables**: Use constexpr lookup tables for transcendental functions
- **SIMD**: Leverage SIMD instructions when available (ARM NEON, CMSIS-DSP)
- **Loop Unrolling**: Manually unroll small fixed-size loops for performance
- **Branch Prediction**: Structure code to favor likely branches first
- **Cache Locality**: Arrange data structures for sequential access patterns
- **Compiler Hints**: Use `inline`, `constexpr`, and `[[likely]]`/`[[unlikely]]` attributes
- **Measure Performance**: Profile before and after optimization

### Numerical Stability

- **Avoid Cancellation**: Restructure equations to avoid subtracting nearly equal numbers
- **Scale Inputs**: Normalize inputs to expected ranges
- **Accumulator Precision**: Use higher precision for accumulation (e.g., Q31 accumulator for Q15 data)
- **Conditional Stability**: Document when algorithms require specific parameter ranges
- **Test Stability**: Include tests with challenging numerical conditions

### API Design Principles

- **Type Safety**: Use strong types and templates to prevent misuse
- **Zero-Cost Abstractions**: Design interfaces that compile to optimal code
- **Compile-Time Configuration**: Use templates for sizes and types when possible
- **Explicit Conversions**: Avoid implicit conversions between numeric types
- **Const Correctness**: Mark all non-mutating operations as const
- **Move Semantics**: Provide move constructors/assignment for large objects (even though we avoid dynamic allocation)

## Compiler Optimizations and SIMD

### Optimization Macros

The library provides compiler-specific optimization hints:

- **OPTIMIZE_FOR_SPEED**: For performance-critical algorithms (PID, filters)
  - GCC: `-O3`, `-ffast-math`, `hot`, `flatten` attributes
  - Clang: `hot`, `flatten` attributes
  - Enables aggressive inlining and optimization
- **ALWAYS_INLINE_HOT**: Forces inlining for frequently-called hot-path code
- **ALWAYS_INLINE**: Forces inlining without hot attribute

### SIMD Support

- Use `math::SingleInstructionMultipltData` interface for SIMD operations
- Provides abstraction over ARM SIMD instructions (DSP, NEON)
- Operations include: packed arithmetic, saturating operations, multiply-accumulate
- Create test doubles (stubs) for host-based testing
- Examples: `Pkhbt`, `Qadd16`, `Smlad`, `Smuad` for efficient fixed-point operations

### Cross-Platform Considerations

- Algorithms must work on both host (x86/x64) and target (ARM Cortex-M/A) platforms
- Use conditional compilation for platform-specific optimizations
- Provide portable fallback implementations
- Test on host before deploying to target hardware
- Be aware of endianness differences when relevant

## Domain-Specific Guidelines

### Signal Processing (Analysis, Filters, Windowing)

- **FFT Implementation**: Prefer radix-2 FFT for power-of-2 sizes
- **Window Functions**: Apply 0.9999 scaling factor to prevent fixed-point overflow
- **Filter Design**: Document frequency response characteristics
- **Coefficient Quantization**: Be aware of quantization effects in fixed-point filters
- **Buffer Management**: Use `RecursiveBuffer` for delay lines and circular buffers
- **Spectral Analysis**: Consider window selection for frequency vs. time resolution trade-offs

### Control Systems (PID, State-Space)

- **Sample Time**: Always specify and document the sampling period
- **Tuning Parameters**: Provide reasonable default ranges and constraints
- **Anti-Windup**: Implement integrator anti-windup for PID controllers
- **Saturation**: Clamp outputs to prevent actuator saturation issues
- **Derivative Filtering**: Consider derivative kick prevention techniques
- **Discretization**: Document the discretization method used (Tustin, forward Euler, etc.)

### Neural Networks

- **Layer Composition**: Use fluent interface for building networks
- **Activation Functions**: Provide both forward and derivative implementations
- **Loss Functions**: Support common losses (MSE, MAE, cross-entropy)
- **Optimization**: Implement gradient-based optimizers (gradient descent, Adam, etc.)
- **Weight Initialization**: Provide proper initialization strategies
- **Fixed-Point Constraints**: Be mindful of dynamic range in activations and weights

### Estimators and Solvers

- **Matrix Operations**: Use efficient algorithms for common operations (LU, Cholesky, etc.)
- **Numerical Conditioning**: Check and document condition number requirements
- **Iterative Methods**: Provide convergence criteria and maximum iteration limits
- **System Identification**: Support AR, MA, ARMA model estimation
- **Kalman Filtering**: Implement measurement and process noise covariance updates

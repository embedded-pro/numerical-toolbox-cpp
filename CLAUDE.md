# Numerical Toolbox — Claude Instructions

## Project Overview

This is a numerical algorithms library providing DSP, control algorithms, filters, optimizers, and estimators for embedded systems. The library targets resource-constrained microcontrollers with strict performance and memory constraints, real-time determinism, and numerical accuracy.

## Repository Structure

- **numerical/**: Core algorithm implementations organized by domain:
  - `analysis/`: Signal analysis (FFT, PSD, DCT)
    - `windowing/`: Window functions (`windowing::` namespace)
  - `control_analysis/`: Control system analysis (Frequency Response, Root Locus)
  - `controllers/implementations/`: Control algorithms (PID, LQR, LQG, MPC)
  - `controllers/interfaces/`: Abstract controller interfaces
  - `dynamics/`: Dynamics and modeling (Euler-Lagrange, Newton-Euler)
  - `estimators/Estimator.hpp`: Base estimator interface
  - `estimators/offline/`: Batch estimators (Linear Regression, Yule-Walker, Expectation Maximization)
  - `estimators/online/`: Streaming estimators (Recursive Least Squares)
  - `filters/active/`: Active filters (Kalman, Extended Kalman, Unscented Kalman, Smoother) — `filters::` namespace
  - `filters/passive/`: Passive filters (FIR, IIR) — `filters::passive::` namespace
  - `kinematics/`: Forward/inverse kinematics
  - `math/`: Mathematical foundations (QNumber, Matrix, Complex, Trigonometric, SIMD, CompilerOptimizations)
  - `neural_network/`: Neural network (activation, layer, losses, model)
  - `optimization/`: General-purpose optimizers (Gradient Descent, Bayesian)
  - `regularization/`: Regularization (L1, L2)
  - `solvers/`: Numerical solvers (Levinson-Durbin, Gaussian Elimination, DARE, Durand-Kerner, Cholesky)
- **simulator/**: Qt-based simulators for algorithm visualization
- **doc/**: Algorithm documentation (mathematical background, not implementation details)
- **cmake/NumericalHeaderLibrary.cmake**: Reusable CMake helpers for all `numerical/` targets

## Critical Constraints

### Memory — NO HEAP ALLOCATION

Never use `new`, `delete`, `malloc`, `free`, `std::make_unique`, or `std::make_shared`.

Replace standard containers:
- `std::vector<T>` → `infra::BoundedVector<T>::WithMaxSize<N>`
- `std::string` → `infra::BoundedString::WithStorage<N>`
- `std::deque<T>` → `infra::BoundedDeque<T>::WithMaxSize<N>`
- `std::list<T>` → `infra::BoundedList<T>::WithMaxSize<N>`
- Fixed-size arrays: `std::array<T, N>`
- Optional values: `std::optional<T>`
- No recursion — stack usage must be predictable

This rule applies to **production code and test code equally**.

### Numeric Types — Template Support

Every algorithm MUST be templated to support all three numeric representations:
- `float` — floating-point
- `math::Q15` — 16-bit fixed-point (range: [-1, ~1], 15 fractional bits)
- `math::Q31` — 32-bit fixed-point (range: [-1, ~1], 31 fractional bits)

Use `std::numbers::pi_v<float>` — never hardcode `3.14159265f`.

Exception: dynamics algorithms (Euler-Lagrange, Newton-Euler) use `float` only because torques/forces typically exceed Q15/Q31 range.

### Fixed-Point Safety

- Analyze overflow risk for Q15/Q31 intermediate values
- Use saturating arithmetic where appropriate
- Use higher-precision accumulators (e.g., Q31 for Q15 data)
- Apply 0.9999 scaling factor for window functions to prevent overflow

### Embedded Optimizations — MANDATORY for algorithm headers

Every header file implementing algorithms under `numerical/` MUST include:

```cpp
#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
```

Apply `OPTIMIZE_FOR_SPEED` (from `numerical/math/CompilerOptimizations.hpp`) on hot-path methods: `Compute()`, `Filter()`, `Calculate()`, `Solve()`, `Update()`, `Step()`.

Pure interface/base headers that contain no algorithm logic are exempt from this requirement.

### Performance Requirements

- Real-time determinism — no blocking, no dynamic branching in hot paths
- Avoid virtual calls in ISR-callable or real-time critical paths
- Use `inline` for small, frequently-called functions
- Use `constexpr` for compile-time calculations and lookup tables
- Mark all non-mutating methods `const`
- Use fixed-size types: `uint8_t`, `int32_t`, etc.

## Namespaces

- `analysis::` — Signal analysis
- `windowing::` — Window functions (under `numerical/analysis/windowing/`)
- `control_analysis::` — Control system analysis
- `controllers::` — Control algorithms
- `dynamics::` — Dynamics and modeling
- `estimators::` — Estimation algorithms
- `filters::` — Active filters (Kalman family); **note: no `filters::active::` sub-namespace exists in code**
- `filters::passive::` — Passive filters (FIR, IIR)
- `math::` — Mathematical utilities
- `neural_network::` — Neural network components
- `optimization::` — General-purpose optimizers
- `regularization::` — Regularization techniques
- `solvers::` — Numerical solvers

## Naming Conventions

- **Classes/Methods**: `PascalCase` — `FirFilter`, `Compute()`, `Reset()`
- **Member variables**: `camelCase` — `sampleRate`, `coefficients`
- **Namespaces**: lowercase — `filters`, `controllers`, `math`
- **Template parameters**: descriptive — `typename T`, `std::size_t Order`
- No abbreviations — use full descriptive names

## Code Style

- Allman braces (opening brace on new line), 4-space indent
- Strictly follow `.clang-format` rules
- Functions ~30 lines max (hard limit ~50 lines)
- Self-documenting code — avoid unnecessary comments
- **Brace initialization**: Use `{}` for all variable and object initialization
  - `T value{}` not `T value()`
  - `Foo obj{arg}` not `Foo obj(arg)`
  - Parenthesis `()` only when brace initialization causes narrowing conversion or parsing ambiguity
- No `TODO`, `FIXME`, or `HACK` comments in production code
- No function/method docstrings unless the API is non-obvious to a domain expert

## Design Principles

### SOLID

- **SRP**: Each class owns exactly one concern. Separate configuration into sub-structs.
- **OCP**: Extend via templates and compile-time polymorphism for new numeric types.
- **LSP**: All derived classes must be fully substitutable; override every pure virtual.
- **ISP**: Keep interfaces small and focused.
- **DIP**: Constructor injection for dependencies; depend on abstractions.

### DRY

- Never duplicate logic. Extract shared code into helpers or templates.
- Use `std::numbers::pi_v<float>` consistently — never hardcode constants.
- Reuse `TimeSeriesChartWidget`, `SignalGenerator`, `RecursiveBuffer`, etc.

### RAII

Use Resource Acquisition Is Initialization for all resource management.

### Interfaces

- Define interfaces (pure virtual classes) for testability
- **NEVER declare `virtual ~Interface() = 0`** — pure virtual destructors force thunk emission in embedded binaries. Use `virtual ~Interface() = default` or omit if never deleted polymorphically.
- No virtual calls in ISR-callable or real-time critical paths.

### Error Handling

- `std::optional<T>` for functions that may not return a value
- Return error codes or status enums — **NO EXCEPTIONS**
- `assert()` or `really_assert()` for precondition checks in debug builds

## Testing

### Framework

- GoogleTest (`TYPED_TEST`, `TEST_F`)
- GoogleMock for mocking (`testing::StrictMock<>`) when needed
- No heap allocation in tests — same no-heap rules as production code

### Test Macro Rules — cppcheck compliance

- **NEVER use plain `TEST()` macro** — cppcheck cannot parse it and will report `syntaxError`
- **Use `TYPED_TEST`** for multi-type tests (float, Q15, Q31)
- **Use `TEST_F`** for single-type fixture tests
- Fixture class and type aliases inside anonymous `namespace {}`
- Test macros (`TEST_F`, `TYPED_TEST`) **outside** the anonymous namespace
- Include `<gtest/gtest.h>` unless gmock matchers are needed
- **Only `StrictMock`**: `testing::StrictMock<MockType>` for ALL mocks — `testing::NiceMock<>` and bare `Mock<>` are forbidden

### Typed Test Pattern (multiple numeric types)

```cpp
#include "numerical/solvers/GaussianElimination.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class TestAlgorithm : public ::testing::Test
    {
    protected:
        solvers::GaussianElimination<T, 3> solver;
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestAlgorithm, TestTypes);
}

TYPED_TEST(TestAlgorithm, computes_correct_result)
{
    // Use TypeParam for the type, this->solver for fixture members
}
```

### Fixture Test Pattern (single type)

```cpp
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestDare : public ::testing::Test
    {
    protected:
        solvers::DiscreteAlgebraicRiccatiEquation<float, 2, 1> solver;
    };
}

TEST_F(TestDare, solves_simple_system)
{
    // Arrange, Act, Assert
}
```

### TDD Approach

- Clarify all requirements as test cases before writing implementation code
- Red-Green-Refactor cycle
- Test numerical accuracy against known reference values (not just "doesn't crash")
- Test edge cases: zero input, maximum range values, saturation conditions

### Coverage for Template Code

When `EMIL_ENABLE_COVERAGE` is set, add explicit template instantiation:

1. In the header (bottom, guarded):
```cpp
#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
extern template class FirFilter<float, 8>;
extern template class FirFilter<math::Q15, 8>;
extern template class FirFilter<math::Q31, 8>;
#endif
```

2. In matching `.cpp` file with explicit instantiation.
3. Add the `.cpp` to `CMakeLists.txt` via `numerical_add_coverage_sources()`.

## Build System

- CMake with presets: `host` (simulation/testing), `coverage` (code coverage), embedded targets
- Build: `cmake --preset host && cmake --build --preset host`
- Test: `ctest --preset host`
- Coverage: `cmake --preset coverage && cmake --build --preset coverage`

### CMake Pattern for `numerical/` Targets

```cmake
numerical_add_header_library(numerical.domain)

target_include_directories(numerical.domain ${NUMERICAL_VISIBILITY}
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(numerical.domain ${NUMERICAL_VISIBILITY}
    infra.util
    numerical.math
)

target_sources(numerical.domain PRIVATE
    Algorithm.hpp
)

numerical_add_coverage_sources(numerical.domain
    Algorithm.cpp
)

add_subdirectory(test)
```

- Use `numerical_add_header_library()` — not raw `add_library()`
- Use `numerical_add_coverage_sources()` for explicit instantiation `.cpp` files
- Use `${NUMERICAL_VISIBILITY}` for `target_include_directories` and `target_link_libraries`

### Simulator CMake Structure

Every simulator follows: `application/` (logic) → `view/` (Qt GUI) → `Main.cpp` (entry point).

Target naming: `numerical.simulator.<domain>.<algorithm>.<layer>`

Every new simulator MUST have a corresponding entry in `.vscode/launch.json` following the existing `cppdbg` pattern. Insert the new entry **before** the generic `"Linux Debug"` configuration.

## Documentation

- **ALWAYS update documentation** when implementing or modifying algorithms
- Location: `doc/{domain}/{AlgorithmName}.md`, following `doc/TEMPLATE.md` exactly
- Documentation is **design-first**: mathematical theory, algorithm behaviour, complexity, pitfalls, connections
- **Do NOT include** implementation details (class names, template parameters, header paths)
- **Do NOT include** usage examples or code snippets
- Keep documentation in sync with algorithmic changes (not code-level changes)

## SIMD Support

- Use `math::SingleInstructionMultipleData` interface for SIMD operations (note correct spelling)
- Provides abstraction over ARM SIMD instructions (DSP, NEON)
- Create test doubles (`numerical/math/test_doubles/SingleInstructionMultipleDataStub.hpp`) for host-based testing

## Known Code Inconsistencies

The following inconsistencies between guidelines and current implementation have been identified:

1. **`filters::active::` namespace** — Instructions historically referred to `filters::active::`, but the actual code uses `namespace filters` (not `namespace filters::active`) for KalmanFilter and related classes. Use `namespace filters` for active filters.

2. **`TestWindowing.cpp`** — Uses `std::vector<std::unique_ptr<...>>` and `std::make_unique<>` in the `WindowSymmetry` test (line ~108), violating the no-heap rule. This needs refactoring to use `std::array` of concrete objects.

3. **`TestStatistics.cpp`** — Uses plain `TEST()` macro (line 128), which is forbidden. Should be converted to `TEST_F`.

4. **Coverage build directory** — The CMake `coverage` preset creates `build/coverage` (lowercase). The devcontainer `gcovViewer.buildDirectories` must match this path.

5. **Missing `#pragma GCC optimize`** — `FastFourierTransform.hpp` and several interface-only headers are missing the pragma. The pragma is required for algorithm implementation headers; pure interface headers are exempt.

## Version Control

- Keep commits atomic and focused
- Use release-please conventions for `CHANGELOG.md`
- When modifying algorithms, update corresponding `doc/` documentation

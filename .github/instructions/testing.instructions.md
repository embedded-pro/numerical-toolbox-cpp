---
description: "Numerical Toolbox testing guidelines: TYPED_TEST for multi-type tests (float/Q15/Q31), TEST_F for fixture tests, anonymous namespace for fixtures, no plain TEST() macro, numerical accuracy validation, Arrange-Act-Assert pattern."
applyTo: "**/test/**"
---

# Numerical Toolbox Testing Guidelines

## File Structure

- Test files: `numerical/{domain}/test/Test{ComponentName}.cpp`
- CMake: tests added via `add_subdirectory(test)` with standard test target patterns

## Framework

- GoogleTest for assertions (`TYPED_TEST`, `TEST_F`)
- GoogleMock for mocking (`testing::StrictMock<>`) when needed
- No heap allocation in tests — same rules as production code
- **NEVER use plain `TEST()` macro** — cppcheck reports `syntaxError`

## Typed Test Pattern (multiple numeric types)

```cpp
#include "numerical/filters/passive/FirFilter.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class TestFirFilter : public ::testing::Test
    {
    protected:
        filters::passive::FirFilter<T, 8> filter;
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFirFilter, TestTypes);
}

TYPED_TEST(TestFirFilter, produces_correct_output_for_known_input)
{
    // Arrange
    // Act
    // Assert
}
```

## Fixture Test Pattern (single type)

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

## Rules

- Fixture class and type aliases go inside anonymous `namespace {}`
- Test macros (`TEST_F`, `TYPED_TEST`) go **outside** the anonymous namespace
- Include `<gtest/gtest.h>` (not `<gmock/gmock.h>`) unless gmock matchers are needed
- Use `testing::StrictMock<MockType>` for strict mock expectations
- **ONLY `StrictMock`**: Never use `testing::NiceMock<>` or bare mock instantiation — `NiceMock` silences unexpected-call warnings, masking test gaps; `StrictMock` enforces all interactions explicitly
- Test all three numeric types: `float`, `math::Q15`, `math::Q31`
- Test numerical accuracy against known reference values (not just "doesn't crash")
- Test edge cases: zero input, maximum range values, saturation conditions
- Test one behavior per test — keep tests focused
- Use descriptive test names that explain the scenario
- Allman brace style and PascalCase naming apply to test code too

## TDD Approach

- **Clarify requirements first**: Before writing any code, define and document all use cases, inputs, outputs, and edge cases as test cases
- **Write tests before implementation**: Tests define the expected behavior; implementation exists only to satisfy the tests
- **Red-Green-Refactor cycle**: Write a failing test, make it pass with minimal code, then refactor while keeping tests green

## Coverage for Template Code

When `EMIL_ENABLE_COVERAGE` is set, template code needs explicit instantiation in a `.cpp` file that is compiled with coverage flags. Add to the header (guarded):

```cpp
#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
extern template class FirFilter<float, 8>;
extern template class FirFilter<math::Q15, 8>;
extern template class FirFilter<math::Q31, 8>;
#endif
```

And in the matching `.cpp` file:

```cpp
#include "numerical/filters/passive/FirFilter.hpp"

namespace filters::passive
{
    template class FirFilter<float, 8>;
    template class FirFilter<math::Q15, 8>;
    template class FirFilter<math::Q31, 8>;
}
```

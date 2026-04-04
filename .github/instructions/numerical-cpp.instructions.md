---
description: "Numerical C++ coding rules for numerical-toolbox: no heap allocation, bounded containers, template-based numeric types (float/Q15/Q31), embedded compiler optimizations, fixed-point arithmetic safety, Allman brace style, PascalCase naming, SOLID principles, const correctness."
applyTo: "**/*.{hpp,cpp,h}"
---

# Numerical Toolbox C++ Rules

This project is a numerical algorithms library for DSP, control algorithms, filters, optimizers, and estimators targeting resource-constrained embedded systems. Follow these rules strictly.

## Memory — No Heap Allocation

Never use `new`, `delete`, `malloc`, `free`, `std::make_unique`, or `std::make_shared`.

Replace standard containers:
- `std::vector<T>` → `infra::BoundedVector<T>::WithMaxSize<N>`
- `std::string` → `infra::BoundedString::WithStorage<N>`
- `std::deque<T>` → `infra::BoundedDeque<T>::WithMaxSize<N>`
- `std::list<T>` → `infra::BoundedList<T>::WithMaxSize<N>`
- Use `std::array<T, N>` for fixed-size arrays
- Use `std::optional<T>` for optional values
- No recursion — stack usage must be predictable

## Numeric Types — Template Support

Every algorithm MUST be templated to support all three numeric representations:
- `float` — floating-point
- `math::Q15` — 16-bit fixed-point (range: [-1, ~1])
- `math::Q31` — 32-bit fixed-point (range: [-1, ~1])

Use `std::numbers::pi_v<float>` — never hardcode `3.14159265f`.

## Fixed-Point Safety

- Analyze overflow risk for Q15/Q31 intermediate values
- Use saturating arithmetic where appropriate
- Use higher-precision accumulators (e.g., Q31 for Q15 data)
- Apply 0.9999 scaling for window functions to prevent overflow

## Embedded Optimizations

Every algorithm header MUST include:

```cpp
#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
```

Apply `OPTIMIZE_FOR_SPEED` (from `numerical/math/CompilerOptimizations.hpp`) on hot-path methods: `Compute()`, `Filter()`, `Calculate()`, `Solve()`, `Update()`, `Step()`.

## Naming

- Classes/Methods: `PascalCase` (e.g., `FirFilter`, `Compute()`)
- Member variables: `camelCase` (e.g., `sampleRate`, `coefficients`)
- Namespaces: lowercase (`filters`, `controllers`, `analysis`, `math`)
- Template parameters: descriptive (`typename T`, `std::size_t Order`)

## Style

- Allman braces (opening brace on new line), 4-space indent
- Functions ~30 lines max (hard limit ~50)
- Self-documenting code — avoid unnecessary comments
- `const` on all non-mutating methods, `constexpr` where possible
- Fixed-size types: `uint8_t`, `int32_t`, etc.

## Design

- SOLID principles — constructor injection, depend on abstractions
- DRY — extract shared logic into helpers or templates
- RAII for resource management
- No virtual calls in ISR-callable or real-time critical paths

## Documentation — MANDATORY

For every algorithm added or modified, update the corresponding `doc/{domain}/{AlgorithmName}.md` file. Include mathematical background, implementation details, usage examples, and numerical properties.

Full details: [copilot-instructions.md](../../.github/copilot-instructions.md)

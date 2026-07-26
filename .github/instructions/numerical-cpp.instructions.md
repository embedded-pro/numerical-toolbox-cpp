---
description: "Numerical C++ rules (float-only): no heap, bounded containers, generic template<typename T> instantiated for float, embedded pragmas, Allman/brace-init, SOLID, const-correct. Canonical: AGENTS.md."
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

## Numeric Types — Float-Only

Write generic `template<typename T[, std::size_t N]>` with
`static_assert(std::is_floating_point_v<T>, "...")`, and **instantiate/test `float` only**.
Do not implement `Q15`/`Q31` — the generic `T` keeps that a cheap future add.
Use `std::numbers::pi_v<float>` — never hardcode `3.14159265f`.

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
- **Brace initialization**: Use `{}` for all variable and object initialization — `T value{}` not `T value()`, `Foo obj{arg}` not `Foo obj(arg)`. Parenthesis `()` may be used only when brace initialization causes a narrowing conversion or ambiguity.

## Design

- SOLID principles — constructor injection, depend on abstractions
- DRY — extract shared logic into helpers or templates
- RAII for resource management
- **No pure virtual destructors**: Never declare `virtual ~Interface() = 0`. Pure virtual destructors force the compiler to emit a thunk and a separate body, wasting memory on embedded targets. Use `virtual ~Interface() = default` if a virtual destructor is required, or omit it entirely when the class is never deleted through a base pointer.
- No virtual calls in ISR-callable or real-time critical paths

## Documentation — MANDATORY

For every algorithm added or modified, update the corresponding `doc/{domain}/{AlgorithmName}.md` file. Follow `doc/TEMPLATE.md` exactly. Documentation is **design-first**: cover mathematical background, algorithm behaviour, complexity, pitfalls, and connections. Do **not** include implementation details, class names, template parameters, or usage code examples — docs describe the algorithm design; code follows from it.

Canonical rules: [AGENTS.md](../../AGENTS.md).

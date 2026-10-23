---
description: "Numerical C++ rules (float-only): no heap, bounded containers, generic template<typename T> instantiated for float, forced inlining on hot paths, Allman/brace-init, SOLID, const-correct. Canonical: AGENTS.md."
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

## Math Functions

Use `math::Sin`, `math::Cos`, `math::Abs`, `math::Sqrt`, etc. from
`numerical/math/Math.hpp` in all production code.
Never call `std::sin`, `std::cos`, `std::abs`, `std::sqrt`, or any other `<cmath>` function directly in production headers or source files.
If a required `<cmath>` function is not yet wrapped in `Math.hpp`, add it there first — then use `math::FunctionName` at the call site.

Each function is guarded by `#ifndef MATH_<NAME>_OVERRIDE` (e.g. `MATH_SIN_OVERRIDE`).
To replace a function with a platform-specific implementation, define the corresponding macro
(e.g. `add_compile_definitions(MATH_SIN_OVERRIDE)` in CMake) and provide your own
`template<typename T> constexpr T math::Sin(T x)` in a header that is included before any call site.

## Embedded Optimizations

Never use `#pragma GCC optimize` or the `optimize` attribute. GCC does not inline a callee whose optimization options differ from its caller's, so options set per header or per function turn every small helper on a hot path into an out-of-line call. Optimization level and floating-point flags are the consumer's, applied to whole translation units (embedded: `-O2`/`-O3` with `-ffast-math -fno-finite-math-only`).

Apply `OPTIMIZE_FOR_SPEED` (from `numerical/math/CompilerOptimizations.hpp`) on hot-path methods: `Compute()`, `Filter()`, `Calculate()`, `Solve()`, `Update()`, `Step()`. It forces inlining when `NumericalToolbox_ENABLE_OPTIMIZATIONS` is defined and expands to nothing otherwise.

Use `math::IsFinite` for finiteness checks: a consumer may still build with `-ffinite-math-only`.

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
- **Protected, non-virtual interface destructors**: declare `~Interface() = default` in a `protected:` section, never `virtual ~Interface()` (defaulted or `= 0`). A virtual destructor puts a deleting destructor in every implementation's vtable, which links `operator delete` (and with it `malloc`/`free`) into heap-free firmware, and under multiple inheritance adds a thunk per base. The protected destructor makes deletion through an interface pointer a compile error. Concrete classes declare no destructor unless they need one, and callers hold the concrete type rather than a `std::unique_ptr` to the interface.
- No virtual calls in ISR-callable or real-time critical paths

## Documentation — MANDATORY

For every algorithm added or modified, update the corresponding `doc/{domain}/{AlgorithmName}.md` file. Follow `doc/TEMPLATE.md` exactly. Documentation is **design-first**: cover mathematical background, algorithm behaviour, complexity, pitfalls, and connections. Do **not** include implementation details, class names, template parameters, or usage code examples — docs describe the algorithm design; code follows from it.

Canonical rules: [AGENTS.md](../../AGENTS.md).

# Numerical Toolbox — Agent Rules (canonical)

Single source of truth for **Claude and Copilot**. `CLAUDE.md` and
`.github/copilot-instructions.md` point here. Code-file specifics load on demand from
`.github/instructions/`. Deployment recipe: `roadmap/DEPLOYMENT.md`.

Numerical algorithms library (DSP, control, filters, estimators, kinematics/dynamics) for
resource-constrained embedded systems. Real-time, deterministic, no heap.

## Numeric policy — FLOAT-ONLY (current)

- New algorithms are **float-only**. Write generic `template<typename T>`, add
  `static_assert(std::is_floating_point_v<T>, "...")`, and **instantiate/test `float` only**.
- Do **NOT** implement `Q15`/`Q31` now. The generic `T` keeps a fixed-point specialisation cheap later.
- `std::numbers::pi_v<float>` — never hardcode constants.

## Memory — no heap

- Forbidden: `new`/`delete`/`malloc`/`free`, `make_unique`/`make_shared`,
  `std::vector`/`string`/`deque`/`list`/`map`/`set`.
- Use: `infra::BoundedVector<T>::WithMaxSize<N>`, `infra::BoundedString::WithStorage<N>`,
  `infra::BoundedDeque<T>::WithMaxSize<N>`, `infra::BoundedList<T>::WithMaxSize<N>`,
  `std::array<T,N>`, `std::optional<T>`. Stack/static only. No recursion. **Tests too.**

## Embedded optimizations (algorithm headers)

```cpp
#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"
```

`OPTIMIZE_FOR_SPEED` on hot paths (`Filter/Compute/Update/Solve/Step`). Pure interfaces exempt.

## Style

- Allman braces, 4-space indent, `.clang-format` authoritative.
- Brace-init `{}` everywhere. PascalCase types/methods, camelCase members, lowercase namespaces.
- Functions ≤ ~30 lines. `const`/`constexpr`-correct. Fixed-width ints. Full descriptive names.
- **No comments** except license headers, `NOLINT`, and a brief note on genuinely non-obvious math.

## Interfaces & errors

- Interfaces = pure virtual classes; `virtual ~I() = default` — **never** `= 0` destructors.
- No exceptions. Use `std::optional<T>` or status enums. `assert`/`really_assert` for preconditions.
- SOLID + DIP: constructor injection; depend on abstractions. RAII. No virtual calls in real-time paths.

## Namespaces

`analysis`, `windowing`, `control_analysis`, `controllers`, `dynamics`, `estimators`,
`filters` (active Kalman family — **not** `filters::active`), `filters::passive`, `math`,
`neural_network`, `optimization`, `regularization`, `solvers`, and new: `trajectory`,
`robust_control`, `nonlinear_control`.

## Testing

- GoogleTest. **`TEST_F` on `float`** — no `TYPED_TEST`, no multi-type. **Never plain `TEST()`**.
- **`StrictMock` only** (no `NiceMock`/bare). Fixture + aliases in an anonymous namespace; macros outside it.
- **No redundant tests** — implement exactly the spec's enumerated cases, one behavior per test,
  Arrange/Act/Assert, `EXPECT_NEAR` + `math::Tolerance<float>()`. No heap.

## CMake (numerical/ targets)

- `numerical_add_header_library(<target>)`, `numerical_add_coverage_sources(<target> <Name>.cpp)`,
  `${NUMERICAL_VISIBILITY}`.
- Coverage `.cpp`: `template class <Name><float, ...>;`. `extern template` guarded by
  `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.

## Docs

Design-first `doc/<domain>/<Name>.md` per `doc/TEMPLATE.md`. Math/theory/complexity/pitfalls —
no code, no class names, no usage examples. Update `doc/<domain>/README.md` when adding a new algorithm.

## Assistant behavior — be terse

- Minimal prose. No preamble/postamble, no restating the plan, no summaries unless asked.
- Report results as file paths + pass/fail. Don't narrate routine tool calls.
- Don't re-read files already read; batch reads; prefer targeted edits.
- Build: `cmake --preset host && cmake --build --preset host` · Test: `ctest --preset host`.

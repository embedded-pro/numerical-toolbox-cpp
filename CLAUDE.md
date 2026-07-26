# Numerical Toolbox — Claude Instructions

Canonical rules: **[AGENTS.md](AGENTS.md)** (shared with Copilot).
Code-file specifics: `.github/instructions/`. Deployment recipe: `roadmap/DEPLOYMENT.md`.

Essentials (full detail in AGENTS.md):
- **No heap** — bounded containers / `std::array` / `std::optional`; no recursion; tests too.
- **Float-only** — generic `template<typename T>` + `static_assert(std::is_floating_point_v<T>)`; instantiate/test `float`; no Q15/Q31.
- **Embedded** — `#pragma GCC optimize("O3","fast-math")` + `OPTIMIZE_FOR_SPEED` on hot paths.
- **No comments** (except license/NOLINT). Allman braces, brace-init, PascalCase/camelCase.
- **Tests** — `TEST_F` on `float`, `StrictMock` only, never plain `TEST()`, no redundant cases.
- **No exceptions** — `std::optional`/status enums; interfaces `virtual ~I() = default`.
- **Be terse** — minimal prose; report file paths + pass/fail.

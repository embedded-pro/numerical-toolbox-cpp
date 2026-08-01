# Deploying a Roadmap Algorithm (float-only recipe)

Turn one `roadmap/<domain>/<Name>/` spec into shipped code. Rules: `../AGENTS.md`.
Read the spec's three files first (`implementation.md`, `tests.md`, `explanation.md`), then:

1. **Header** `numerical/<domain>/<Name>.hpp`
   - `#pragma once` → `#pragma GCC optimize("O3","fast-math")` (GCC/Clang guard) →
     `#include "numerical/math/CompilerOptimizations.hpp"`.
   - `template<typename T[, std::size_t sizes...]>` with
     `static_assert(std::is_floating_point_v<T>, "<Name> supports floating-point types");`.
   - Implement per `implementation.md`; `OPTIMIZE_FOR_SPEED` on the hot path(s).
   - Bottom: `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD` / `extern template class <Name><float, ...>;` / `#endif`.

2. **Coverage** `numerical/<domain>/<Name>.cpp`
   - Include the header; `namespace <ns> { template class <Name><float, ...>; }`.

3. **Test** `numerical/<domain>/test/Test<Name>.cpp`
   - Per `tests.md`. `TEST_F` on `float`; `StrictMock` only; fixture in an anonymous namespace.
   - Implement **exactly** the enumerated cases — no extra/redundant tests. `EXPECT_NEAR` +
     `math::Tolerance<float>()` (or explicit tol) against known reference values.

4. **CMake**
   - Add `.hpp` to `target_sources(...)`, `.cpp` to `numerical_add_coverage_sources(...)`,
     `Test<Name>.cpp` to the `_test` target's `target_sources`.
   - New module (`robust_control`, `nonlinear_control`):
     create `numerical/<module>/CMakeLists.txt` via `numerical_add_header_library(...)`, add a
     `test/` subdir, register it in the parent `CMakeLists.txt`, and add a `doc/<module>/` folder.

5. **Doc** `doc/<domain>/<Name>.md` per `doc/TEMPLATE.md` (design-first; no code/class names/usage).
   Add its row to `doc/<domain>/README.md` and to `README.md`'s Documentation table — these README
   tables are the booklet's ordering source, so the booklet (`scripts/build-booklet.py`) regenerates
   automatically in CI; never edit the booklet by hand.

6. **Build & test**, fix until green:
   `cmake --preset host && cmake --build --preset host && ctest --preset host`
   (scope to the target/test where possible).

7. **Remove roadmap spec** — delete the entire `roadmap/<domain>/<Name>/` directory once all tests are green.

**Report**: the file paths created/edited/deleted + the test result. Nothing else.

Recap: float-only (generic `T`, `float` instantiation), no heap, no comments, terse.

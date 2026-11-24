# numerical-toolbox-cpp — Agent Rules

<!-- shared:begin 0.5.0 sha256:9aee198041110762 -->
Canonical rules for Claude, Copilot and sub-agents. Sections marked *(shared)* come from embedded-pro/embedded-agent-kit:
edit them there, not here. *(repo)* sections belong to this repository. *Exceptions* overrides a shared rule.

## Behaviour *(shared)*

- **Be as terse as possible.** No preamble, summary or narration. Report file paths and pass/fail.
- Everything you write — code, docs, agent files, PR text — uses the fewest words that stay unambiguous.
- Prose (docs, comments, agent files, commit and PR text) follows ASD-STE100 Simplified Technical English:
  one word, one meaning; active voice; one instruction per sentence; sentences ≤ 20 words (procedures) or ≤ 25 (descriptions).
- Ambiguous request: ask. Clear request: act.
- Do only what was asked. No unrelated refactors, no extra features.
- Batch reads, don't re-read, prefer targeted edits.
- Never change a shared section. Add a deviation to *Exceptions* only when the user approves it.

## Workflow *(shared)*

1. Multi-file or architectural change: update the design document first. Get approval before you write code.
2. Read *Domain* and *Build*. Read each `.github/instructions/*.instructions.md` whose `applyTo` matches a file you change.
3. Find the nearest existing pattern. Follow it.
4. Per behaviour: write a failing test, write the minimum code that passes it, refactor with all tests green.
5. Add each new file to its `CMakeLists.txt`.
6. Update the documentation for each behavioural change.
7. Run the *Build* commands until build and tests pass. Never disable, skip or weaken a test to get green.
8. Non-trivial change: run the `reviewer` agent, or ask the user to run it. Fix each CRITICAL and WARNING finding.
9. Report changed files, build pass/fail, test pass/fail.

## Memory *(shared)*

Scope: production code. Tests and host tools may use the heap.

- Forbidden: `new`/`delete`/`malloc`/`free`, `make_unique`/`make_shared`,
  `std::vector`/`string`/`deque`/`list`/`map`/`set`, recursion.
- Use: `infra::BoundedVector<T>::WithMaxSize<N>`, `infra::BoundedString::WithStorage<N>`,
  `infra::BoundedDeque`, `infra::BoundedList`/`IntrusiveList`, `std::array`, `std::optional`, `std::variant`.

## Interfaces & errors *(shared)*

- No virtual destructors: each adds a deleting destructor and thunks to flash.
  Interface = pure virtual methods + `protected: ~I() = default;` (non-virtual).
- Public `virtual ~Base() = default;` only on a base destroyed through a base pointer:
  `delete`/`std::unique_ptr<Base>`, `infra::PolymorphicVariant<Base, ...>`, `infra::OptionalForPolymorphicObjects<Base, ...>`.
  Put it on that base, not on every interface.
- Not needed for `std::variant`, `std::optional<Concrete>`, `infra::SharedPtr`: they destroy the concrete type.
- No exceptions. `std::optional`, `bool` or status enum; `really_assert` for preconditions. Never swallow an error.
- Constructor injection; no global state.

## Optimisation *(shared)*

- No `#pragma GCC optimize`, no `optimize` attribute: GCC won't inline across differing options.
  The consumer sets flags per translation unit.
- `OPTIMIZE_FOR_SPEED` only on hot functions defined in a header.
- Hot paths: no heap, no virtual dispatch, no blocking.

## Style *(shared)*

- `.clang-format` is authoritative. Allman, 4 spaces, `{}` init.
- PascalCase types/methods; camelCase members and enum values; lowercase namespaces.
  No `I` prefix; implementations end in `Impl`.
- `#pragma once` only.
- Fixed-width integers. `const`/`constexpr`-correct. Functions ≤ 30 lines (hard limit 50).
- Comments: non-obvious *why* only, one line. No `TODO`/`FIXME`, commented-out code, or restating the code.
  Fix or delete a comment the change makes stale.

## Testing *(shared)*

- `TEST_F` for fixtures, `TYPED_TEST` for types, plain `TEST()` for stateless checks.
- `StrictMock` only. `EXPECT_NEAR` with explicit tolerance. Fixture in an anonymous namespace, macros outside.
- Heap allowed in tests.

## Documentation *(shared)*

- Update docs before or with any behavioural change.
- One fact, one place: link, never restate.
- Visuals: Mermaid or ASCII.
<!-- shared:end -->

## Domain *(repo)*

- Numerical algorithms (DSP, control, filters, estimators, solvers) for embedded targets. Source root: `numerical/`.
- Float only: generic `template<typename T>` with `static_assert(std::is_floating_point_v<T>)`; instantiate and test
  `float`. No `Q15`/`Q31`. Constants from `std::numbers` (`std::numbers::pi_v<float>`), never literals.
- Production code calls the `numerical/math/Math.hpp` wrappers (`math::Sin`, `math::Abs`, ...), never `<cmath>`.
  A missing wrapper: add a `constexpr` one with its `MATH_<NAME>_OVERRIDE` guard first. Tests call `std::` directly.
- `math::IsFinite` for finiteness checks: a consumer may build with `-ffinite-math-only`.
- `OPTIMIZE_FOR_SPEED` on `Filter`/`Compute`/`Update`/`Solve`/`Step`. `NumericalToolbox_ENABLE_OPTIMIZATIONS` turns it on;
  it is a usage requirement of `numerical.math`, gated by `NUMERICAL_TOOLBOX_ENABLE_OPTIMIZATIONS`. Never move it to
  `add_definitions` or rename it.
- Optimisation flags belong to the consumer: `numerical_toolbox_enable_embedded_optimizations([TARGETS ...])`
  (`cmake/NumericalCompilerOptions.cmake`) adds `-ffast-math -fno-finite-math-only`. Opt-in only, never a usage
  requirement. `NUMERICAL_TOOLBOX_EMBEDDED_OPTIMIZATIONS` applies it to the toolbox's own targets (QEMU presets).
- Namespaces: `analysis`, `windowing`, `control_analysis`, `controllers`, `estimators`, `filters` (Kalman family, not
  `filters::active`), `filters::passive`, `math`, `optimization`, `regularization`, `solvers`, `robust_control`,
  `nonlinear_control`, `motion_profiles`.
- CMake: `numerical_add_header_library(<target>)`, `numerical_add_coverage_sources(<target> <Name>.cpp)`,
  `${NUMERICAL_VISIBILITY}`. Tests: `numerical/<domain>/test/Test<Name>.cpp`.
- Coverage: the `.cpp` holds `template class <Name><float, ...>;`; the header holds the matching `extern template`
  inside `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`. Coverage ≥ 90 % (SonarQube gate).
- Tests: `TEST_F` on `float`, no `TYPED_TEST`. Implement exactly the spec's cases. `EXPECT_NEAR` with
  `math::Tolerance<float>()`. `TESTING.md` sets what to assert.
- Docs: `doc/<domain>/<Name>.md` per `doc/TEMPLATE.md`: math, behaviour, complexity, pitfalls; no code, no class names.
  New algorithm: add it to `doc/<domain>/README.md`.
- Roadmap: `ROADMAP.md`, specs in `roadmap/<domain>/<Name>/`, deployment recipe in `roadmap/DEPLOYMENT.md`
  (skill `algo-implementer`). Tests: skill `unit-tester`.
- New simulator (`simulator/`): add a `cppdbg` entry to `.vscode/launch.json` before `"Linux Debug"`.

## Build *(repo)*

```bash
cmake --preset host && cmake --build --preset host
ctest --preset host
cmake --preset coverage && cmake --build --preset coverage
```

Presets: `host`, `host-single-Debug` (no Qt), `coverage`, `integration`, `qemu-cortex-m4`, `qemu-cortex-m7`.

## Exceptions *(repo)*

- No plain `TEST()`: MegaLinter's cppcheck reports a syntax error on it. Use `TEST_F`.

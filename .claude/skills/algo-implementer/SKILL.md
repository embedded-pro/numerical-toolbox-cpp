---
name: algo-implementer
description: Deploy one algorithm spec from roadmap/ into a toolbox as float-only production code, tests, documentation and CMake. Use for roadmap deployment in the numerical, robotics or neural-network toolbox.
---

Deploy one algorithm from `roadmap/<domain>/<Name>/`. `<root>` is the source folder that AGENTS.md *Domain* names.

## Steps

1. Read `roadmap/DEPLOYMENT.md`. Then read only the spec's `implementation.md`, `tests.md` and `explanation.md`.
2. Write what `roadmap/DEPLOYMENT.md` lists: `<root>/<domain>/<Name>.hpp`, the coverage `.cpp`,
   `<root>/<domain>/test/Test<Name>.cpp`, `doc/<domain>/<Name>.md` and the CMake changes.
3. Write exactly the tests that `tests.md` lists. Add no other tests.
4. Run the *Build* commands. Fix until build and tests pass.
5. Remove the algorithm's row from `ROADMAP.md`. Add it to the category row of the README documentation table
   and to the algorithm table in `doc/<domain>/README.md`. CI builds the booklet from these tables.
6. Report changed files and test pass/fail.

## Rules

- Float only: `template<typename T>` with `static_assert(std::is_floating_point_v<T>)`. Instantiate and test `float`.
  No `Q15`/`Q31`.
- `OPTIMIZE_FOR_SPEED` on hot functions defined in the header.

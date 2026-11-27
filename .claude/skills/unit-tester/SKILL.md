---
name: unit-tester
description: Write metric-driven unit tests for one toolbox algorithm, with assertions against independent references chosen from TESTING.md. Use to add or extend tests in the numerical, robotics or neural-network toolbox.
---

Add or extend the tests of one algorithm. `<root>` is the source folder that AGENTS.md *Domain* names.
`TESTING.md` defines what to assert: metric families M1–M9 and, where it defines them, robustness dimensions R1–R6.
Apply them; do not copy them.

## Steps

1. Read `TESTING.md`, the public interface in `<root>/<domain>/<Name>.hpp`, `doc/<domain>/<Name>.md`
   and `<root>/<domain>/test/Test<Name>.cpp` if it exists.
2. Find the algorithm's family in `TESTING.md`. Select its M metrics. If `TESTING.md` defines R1–R6, apply R1–R3 always and R4–R6 where the family needs them.
3. Write one `TEST_F` on `float` per property. Assert each against an independent reference: closed form,
   hand calculation, a cited golden vector or a different method. Use `EXPECT_NEAR` with `math::Tolerance<float>()`.
4. Add a new test file to `<root>/<domain>/test/CMakeLists.txt`.
5. Run the *Build* commands. Fix until build and tests pass.
6. Check the per-algorithm checklist in `TESTING.md`, if it has one. Coverage must be ≥ 90 %; an uncovered branch
   usually means a missing unhappy-flow test.
7. Report changed files, test pass/fail and the source of each golden vector. Put the sources in the report, not in the test file.

## Rules

- Never assert the implementation against its own output.
- One behaviour per test. No test per parameter permutation.
- Seeded random generators only.

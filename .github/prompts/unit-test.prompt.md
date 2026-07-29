---
description: "Author metric-driven unit tests for one numerical/ algorithm — TEST_F on float, no heap, StrictMock, reference-value assertions selected per family from TESTING.md."
agent: "unit-tester"
argument-hint: "Name the numerical/ algorithm to author unit tests for (e.g. Biquad, KalmanFilter, RungeKuttaIntegrators)"
model: "Claude Sonnet 5"
---

Author (or extend) the unit tests for the named **numerical-toolbox** algorithm in
`numerical/<domain>/test/Test<Name>.cpp`. Follow the `unit-tester` workflow: read the algorithm's
public interface and doc; look up its family in `TESTING.md` and select the metric
types to assert (accuracy, frequency/transient response, stability, boundaries, invariants,
convergence, statistical consistency, conditioning); write one `TEST_F` on `float` per distinct
property against an independent reference value (`EXPECT_NEAR` + `math::Tolerance<float>()`); wire
CMake; build, run, and report paths + pass/fail. Rules: `AGENTS.md` +
`.github/instructions/testing.instructions.md` (float-only, no heap, StrictMock, no redundant cases).

Algorithm to cover:

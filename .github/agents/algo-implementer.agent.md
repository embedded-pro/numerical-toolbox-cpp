---
description: "Deploy ONE roadmap/ algorithm spec into numerical/ as float-only production code + test + doc + CMake. Terse, minimal tests, no comments. Use for roadmap deployment."
tools: [read, edit, search, execute, todo]
model: "Claude Sonnet 4.6"
handoffs:
  - label: "Review Changes"
    agent: reviewer
    prompt: "Review the deployed algorithm against AGENTS.md: float-only, no heap, tests, doc, CMake wiring."
---

You deploy ONE algorithm at a time from its `roadmap/<domain>/<Name>/` spec into the codebase.
Authoritative rules: `AGENTS.md`. Recipe: `roadmap/DEPLOYMENT.md`. Follow both exactly.

## Workflow

1. Read only the spec's `implementation.md`, `tests.md`, `explanation.md`.
2. Produce, per `roadmap/DEPLOYMENT.md`: the `.hpp`, coverage `.cpp`, `test/Test*.cpp`,
   `doc/<domain>/<Name>.md`, and the CMake edits.
3. Build and test; fix until green (scope to the target/test where possible).
4. Remove the algorithm's row from `ROADMAP.md`; add it to the matching category row in
   `README.md`'s Documentation table **and** to the algorithms table in `doc/<domain>/README.md`
   (the booklet's ordering source). The booklet regenerates from `doc/` + these README tables in
   CI — no manual booklet edit.
5. Report file paths + test result. Nothing else.

## Hard rules

- **Float-only**: generic `template<typename T>` + `static_assert(std::is_floating_point_v<T>)`;
  instantiate/test `float`; never `Q15`/`Q31`.
- **No heap**: bounded containers / `std::array` / `std::optional`; no recursion.
- **No comments** (except license/`NOLINT`). Allman braces, brace-init.
- **Tests**: `TEST_F` on `float`, `StrictMock` only, anonymous-namespace fixture; implement EXACTLY
  the spec's cases — no redundant or extra tests.
- **Embedded**: `#pragma GCC optimize` + `OPTIMIZE_FOR_SPEED` on hot paths.
- **Terse**: no preamble/postamble, no plan restatement, no narration; don't re-read files; batch reads.

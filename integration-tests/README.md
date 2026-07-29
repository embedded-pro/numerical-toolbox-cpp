# Integration tests

Behaviour-driven integration tests built on
[amp-cucumber-cpp-runner](https://github.com/philips-software/amp-cucumber-cpp-runner)
(pinned to `v4.1.0`, fetched via CMake `FetchContent`). Scenarios exercise the
library end-to-end; the first suite drives the incremental PID controller.

## Layout

- `features/` — Gherkin `.feature` files (the executable specification).
- `steps/` — C++ step definitions binding Gherkin phrases to library calls.
- `Main.cpp` — the `cucumber_cpp::Application` entry point.

## Running

```bash
cmake --preset integration          # first run fetches the runner (network)
cmake --build --preset integration-Debug
ctest --preset integration
```

Run the compiled runner directly to iterate on steps:

```bash
build/integration/integration-tests/Debug/numerical.integration_test \
    --format pretty -- integration-tests/features
```

In VSCode, use the "PID Integration Tests" launch configuration to debug the runner.

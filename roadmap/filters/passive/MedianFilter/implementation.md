# Median Filter (Sliding Window) — Implementation Pseudocode

> Roadmap ref: #6 (Tier 2) · Target: `numerical/filters/passive` · Namespace `filters::passive` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
template<typename T, std::size_t N>        # static_assert(std::is_floating_point_v<T>); instantiated for float
class MedianFilter:                        # N odd
    array<T, N>  window                    # last N samples in arrival order (ring)
    array<T, N>  scratch                   # sort buffer (no heap)
    index        head = 0                  # ring write position
    count        filled = 0                # samples seen during warm-up
```

## Interface

```
MedianFilter(T initial = 0)
T    Filter(T input)          # hot path
void Reset(T value = 0)
```

## Algorithm (pseudocode)

```
function Filter(x):                        # OPTIMIZE_FOR_SPEED
    window[head] = x                       # overwrite oldest sample
    head = (head + 1) mod N
    scratch = window                       # copy N words
    insertion_sort(scratch)                # bounded, N small & fixed
    return scratch[N / 2]                  # middle element = median
```

## Complexity & memory

- Time: `O(N)` per sample for insertion sort on small fixed `N` (`O(N²)` worst case, but `N` is a
  compile-time constant, typically 3–9).
- Memory: `O(N)` — one ring buffer plus one sort scratch buffer, all stack/static.

## Numerical / embedded notes

- Purely comparison-based: only copies and compares samples, never adds or multiplies, so there is
  **no coefficient quantization and no arithmetic round-off** on the hot path.
- `N` must be odd for a unique centre element; even `N` would require averaging the two central
  samples (introduces arithmetic on the two central samples).
- Non-linear: rejects isolated impulses / salt-and-pepper spikes while preserving step edges that a
  linear average would smear.
- Huang et al. give an `O(1)`-update running-histogram variant; for small embedded windows the
  direct insertion sort is simpler and cache-friendly.
- Warm-up: pre-fill the window with the seed value so early outputs are well defined.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/filters/passive/MedianFilter.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `Filter`, and
  `extern template class MedianFilter<float, N>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/filters/passive/MedianFilter.cpp` →
  `template class MedianFilter<float, N>;`
- Test: `numerical/filters/passive/test/TestMedianFilter.cpp`
- Doc: `doc/filters/passive/MedianFilter.md` (expand to follow `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestMedianFilter.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

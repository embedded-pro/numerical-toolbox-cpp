# Denavit-Hartenberg Parameters — Implementation Pseudocode

> Roadmap ref: #M7 (Tier 2) · Target: `numerical/kinematics` · Namespace `kinematics` · Type: `float` (templated on `T`, instantiated for `float` only)

## Data structures

```
enum JointType: Revolute, Prismatic

template<typename T>                           # static_assert(std::is_floating_point_v<T>); instantiated for float
struct DhLink:
    T         a          # link length   (along xᵢ)
    T         alpha      # link twist    (about xᵢ)
    T         d          # link offset   (along zᵢ₋₁)
    T         theta      # joint angle   (about zᵢ₋₁)
    JointType type       # which of {theta, d} is the variable

template<typename T, std::size_t NumLinks>
class DenavitHartenberg:
    std::array<DhLink<T>, NumLinks> links      # constant frame description
    bool modified = false                      # standard (distal) vs modified (proximal) DH
```

## Interface

```
DenavitHartenberg(std::array<DhLink<T>, NumLinks> links, bool modified = false)
SE3<T>                     LinkTransform(DhLink<T> link, T q)     # one Aᵢ; hot path
SE3<T>                     Forward(JointVector q)                 # ⁰Tₙ; hot path
std::array<SE3<T>, N + 1>  FrameChain(JointVector q)              # every ⁰Tᵢ (for Jacobian)
```

## Algorithm (pseudocode)

```
function LinkTransform(link, q):                # OPTIMIZE_FOR_SPEED
    theta = link.type == Revolute  ? q : link.theta
    d     = link.type == Prismatic ? q : link.d
    (ct, st) = (cos theta, sin theta)
    (ca, sa) = (cos link.alpha, sin link.alpha)   # constant per link — precompute
    # standard (distal) DH:  Rotz(θ)·Transz(d)·Transx(a)·Rotx(α)
    R = [[ ct, -st·ca,  st·sa ],
         [ st,  ct·ca, -ct·sa ],
         [  0,     sa,     ca ]]
    p = (link.a·ct, link.a·st, d)
    return SE3(R, p)

function Forward(q):                            # OPTIMIZE_FOR_SPEED
    T = Identity
    for i in 0..N-1:
        T = T * LinkTransform(links[i], q[i])   # reuse SE3 compose (M6)
    return T

function FrameChain(q):
    frames[0] = Identity
    for i in 0..N-1:
        frames[i+1] = frames[i] * LinkTransform(links[i], q[i])
    return frames
```

## Complexity & memory

- `LinkTransform`: `O(1)` — four trig calls and a fixed 3×3 fill, no loop.
- `Forward` / `FrameChain`: `O(N)` SE(3) products; `FrameChain` stores `N+1` transforms.
- Memory: the constant `NumLinks` link table plus one `SE(3)` accumulator; all on the stack.

## Numerical / embedded notes

- Precompute `sin/cos` of each `alpha` once (constant per link) — only `theta`/`d` vary at runtime.
- Support both **standard** and **modified** DH: the two conventions place the frame differently, so
  the factor order changes; expose the flag rather than hard-coding one.
- `JointType` selects whether `theta` or `d` is the variable — one struct serves revolute and prismatic
  chains (unblocks mixed SCARA/gantry arms), unlike the revolute-only `RevoluteJointLink`.
- Keep each `R` on `SO(3)`: the factored form is exactly orthonormal, so no reorthonormalization needed.
- Float-only: `static_assert(std::is_floating_point_v<T>)`; the generic `T` signature keeps a
  `Q15`/`Q31` specialisation cheap to add later.

## Deployment

- Header: `numerical/kinematics/DenavitHartenberg.hpp` — `#pragma once` →
  `#pragma GCC optimize("O3","fast-math")`, `OPTIMIZE_FOR_SPEED` on `LinkTransform`/`Forward`, and
  `extern template class DenavitHartenberg<float, NumLinks>;` under `#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD`.
- Coverage: `numerical/kinematics/DenavitHartenberg.cpp` → `template class DenavitHartenberg<float, NumLinks>;`
- Test: `numerical/kinematics/test/TestDenavitHartenberg.cpp`
- Doc: `doc/kinematics/DenavitHartenberg.md` (per `doc/TEMPLATE.md`)
- CMake: `.hpp` → `target_sources`; `.cpp` → `numerical_add_coverage_sources`;
  `TestDenavitHartenberg.cpp` → the `_test` target.
- Generic pattern: see `roadmap/DEPLOYMENT.md`.

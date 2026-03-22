# Geometry3D

## Overview & Motivation

`Geometry3D.hpp` provides shared 3D geometric primitives used across the dynamics and kinematics libraries. These are fundamental building blocks for rigid-body algorithms: cross products, rotation matrices, skew-symmetric matrices, and outer products.

By centralizing these operations in the `math` namespace, we eliminate code duplication across [ABA](../dynamics/ArticulatedBodyAlgorithm.md), [RNEA](../dynamics/RecursiveNewtonEuler.md), and [Forward Kinematics](../kinematics/ForwardKinematics.md).

## Type Aliases

| Alias            | Expands To               | Description              |
|------------------|--------------------------|--------------------------|
| `Vector3<T>`     | `math::Vector<T, 3>`     | 3D column vector         |
| `Matrix3<T>`     | `math::SquareMatrix<T, 3>` | 3×3 matrix             |

## Functions

### `CrossProduct(a, b)` → `Vector3<T>`

Standard 3D vector cross product: $a \times b$.

### `SkewSymmetric(v)` → `Matrix3<T>`

Returns the 3×3 skew-symmetric matrix $[v]_\times$ such that $[v]_\times \cdot u = v \times u$:

$$[v]_\times = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}$$

### `RotationAboutAxis(axis, angle)` → `Matrix3<T>`

Rodrigues' rotation formula. Computes the rotation matrix for angle $\theta$ about unit axis $\hat{k}$:

$$R = I + \sin\theta \, [\hat{k}]_\times + (1 - \cos\theta) \, [\hat{k}]_\times^2$$

Equivalent to the expanded form:

$$R_{ij} = \cos\theta \, \delta_{ij} + (1 - \cos\theta) \, k_i k_j - \sin\theta \, \epsilon_{ijk} k_k$$

### `ScaledIdentity(value)` → `Matrix3<T>`

Returns $\alpha I_{3 \times 3}$ — a diagonal matrix with `value` on the diagonal.

### `OuterProduct(a, b)` → `Matrix3<T>`

Returns the 3×3 outer product $a \, b^T$ where $(a \, b^T)_{ij} = a_i \, b_j$.

## Performance

All functions are marked `ALWAYS_INLINE_HOT` and the header applies `#pragma GCC optimize("O3", "fast-math")`, ensuring these primitives are inlined and optimized even in debug builds.

## Connections to Other Algorithms

- Used by [Articulated Body Algorithm](../dynamics/ArticulatedBodyAlgorithm.md) for spatial inertia transforms
- Used by [Recursive Newton-Euler](../dynamics/RecursiveNewtonEuler.md) for velocity/force propagation
- Used by [Forward Kinematics](../kinematics/ForwardKinematics.md) for joint rotation accumulation

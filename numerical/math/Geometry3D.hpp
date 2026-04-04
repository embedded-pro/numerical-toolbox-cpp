#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cmath>

namespace math
{
    template<typename T>
    using Vector3 = Vector<T, 3>;

    template<typename T>
    using Matrix3 = SquareMatrix<T, 3>;

    template<typename T>
    ALWAYS_INLINE_HOT Vector3<T> CrossProduct(const Vector3<T>& a, const Vector3<T>& b)
    {
        return Vector3<T>{
            a.at(1, 0) * b.at(2, 0) - a.at(2, 0) * b.at(1, 0),
            a.at(2, 0) * b.at(0, 0) - a.at(0, 0) * b.at(2, 0),
            a.at(0, 0) * b.at(1, 0) - a.at(1, 0) * b.at(0, 0)
        };
    }

    template<typename T>
    ALWAYS_INLINE_HOT Matrix3<T> SkewSymmetric(const Vector3<T>& v)
    {
        return Matrix3<T>{
            { T(0), -v.at(2, 0), v.at(1, 0) },
            { v.at(2, 0), T(0), -v.at(0, 0) },
            { -v.at(1, 0), v.at(0, 0), T(0) }
        };
    }

    // Rodrigues' rotation formula: R(axis, angle)
    // R = I + sin(theta)*K + (1 - cos(theta))*K^2
    // where K is the skew-symmetric matrix of 'axis'
    template<typename T>
    ALWAYS_INLINE_HOT Matrix3<T> RotationAboutAxis(const Vector3<T>& axis, T angle)
    {
        T c = std::cos(angle);
        T s = std::sin(angle);
        T t = T(1) - c;

        T x = axis.at(0, 0);
        T y = axis.at(1, 0);
        T z = axis.at(2, 0);

        return Matrix3<T>{
            { t * x * x + c, t * x * y - s * z, t * x * z + s * y },
            { t * x * y + s * z, t * y * y + c, t * y * z - s * x },
            { t * x * z - s * y, t * y * z + s * x, t * z * z + c }
        };
    }

    template<typename T>
    ALWAYS_INLINE_HOT Matrix3<T> ScaledIdentity(T value)
    {
        Matrix3<T> result{};
        result.at(0, 0) = value;
        result.at(1, 1) = value;
        result.at(2, 2) = value;
        return result;
    }

    template<typename T>
    ALWAYS_INLINE_HOT Matrix3<T> OuterProduct(const Vector3<T>& a, const Vector3<T>& b)
    {
        Matrix3<T> result{};
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t c = 0; c < 3; ++c)
                result.at(r, c) = a.at(r, 0) * b.at(c, 0);
        return result;
    }

    template<typename T>
    ALWAYS_INLINE_HOT T DotProduct(const Vector3<T>& a, const Vector3<T>& b)
    {
        return a.at(0, 0) * b.at(0, 0) + a.at(1, 0) * b.at(1, 0) + a.at(2, 0) * b.at(2, 0);
    }

    template<typename T>
    ALWAYS_INLINE_HOT T VectorNorm(const Vector3<T>& v)
    {
        return std::sqrt(DotProduct(v, v));
    }
}

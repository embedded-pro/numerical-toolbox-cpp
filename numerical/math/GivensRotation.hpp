#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <type_traits>

namespace math
{
    template<typename T>
    struct GivensRotation
    {
        T c;
        T s;
    };

    template<typename T>
    [[nodiscard]] OPTIMIZE_FOR_SPEED GivensRotation<T> ComputeGivens(T a, T b)
    {
        static_assert(std::is_floating_point_v<T>, "ComputeGivens supports floating-point types only");

        T r = math::Sqrt(a * a + b * b);
        if (r < T{ 1e-30f })
            return { T{ 1 }, T{} };

        return { a / r, b / r };
    }

    template<typename T>
    [[nodiscard]] OPTIMIZE_FOR_SPEED GivensRotation<T> ComputeJacobiRotation(T app, T aqq, T apq)
    {
        static_assert(std::is_floating_point_v<T>, "ComputeJacobiRotation supports floating-point types only");

        if (apq == T{})
            return { T{ 1 }, T{} };

        T theta = (aqq - app) / (T{ 2 } * apq);
        T t = ((theta >= T{}) ? T{ 1 } : T{ -1 }) / (math::Abs(theta) + math::Sqrt(theta * theta + T{ 1 }));
        T c = T{ 1 } / math::Sqrt(t * t + T{ 1 });

        return { c, t * c };
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED void ApplyGivens(const GivensRotation<T>& g, T& x, T& y)
    {
        T newX = g.c * x + g.s * y;
        T newY = -g.s * x + g.c * y;
        x = newX;
        y = newY;
    }
}

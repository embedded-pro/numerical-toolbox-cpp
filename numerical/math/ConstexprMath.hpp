#pragma once

#include <type_traits>

namespace math
{
    template<typename T>
    [[nodiscard]] constexpr T Sqrt(T x) noexcept
    {
        static_assert(std::is_floating_point_v<T>, "Sqrt requires a floating-point type");
        T r = x;
        for (int i = 0; i < 20; ++i)
            r = T{ 0.5 } * (r + x / r);
        return r;
    }
}

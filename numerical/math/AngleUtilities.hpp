#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <numbers>
#include <type_traits>

namespace math
{
    template<typename T>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T WrapToPi(T angle)
    {
        static_assert(std::is_floating_point_v<T>, "WrapToPi supports floating-point types only");

        constexpr T pi{ std::numbers::pi_v<T> };
        constexpr T twoPi{ T{ 2 } * std::numbers::pi_v<T> };

        T wrapped{ math::Fmod(angle + pi, twoPi) };
        if (wrapped <= T{})
            wrapped += twoPi;

        return wrapped - pi;
    }

    template<typename T>
    [[nodiscard]] T WrapToTwoPi(T angle)
    {
        static_assert(std::is_floating_point_v<T>, "WrapToTwoPi supports floating-point types only");

        constexpr T twoPi{ T{ 2 } * std::numbers::pi_v<T> };

        T wrapped{ math::Fmod(angle, twoPi) };
        if (wrapped < T{})
            wrapped += twoPi;

        return wrapped >= twoPi ? T{} : wrapped;
    }

    template<typename T>
    [[nodiscard]] T AngleDifference(T target, T current)
    {
        static_assert(std::is_floating_point_v<T>, "AngleDifference supports floating-point types only");

        return WrapToPi(target - current);
    }

    template<typename T>
    [[nodiscard]] T UnwrapAngle(T previousUnwrapped, T wrapped)
    {
        static_assert(std::is_floating_point_v<T>, "UnwrapAngle supports floating-point types only");

        return previousUnwrapped + AngleDifference(wrapped, previousUnwrapped);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template float WrapToPi<float>(float);
    extern template float WrapToTwoPi<float>(float);
    extern template float AngleDifference<float>(float, float);
    extern template float UnwrapAngle<float>(float, float);
#endif
}

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <algorithm>
#include <cmath>
#include <type_traits>

namespace analysis
{
    template<typename T>
    struct DecibelFloor
    {
        static_assert(std::is_floating_point_v<T>, "DecibelFloor supports floating-point types only");
        static constexpr T value{ T{ -160 } };
    };

    template<typename T>
    OPTIMIZE_FOR_SPEED T ToDecibels(T ratio)
    {
        static_assert(std::is_floating_point_v<T>, "ToDecibels supports floating-point types only");
        if (ratio <= T{ 0 })
            return DecibelFloor<T>::value;
        return std::max(T{ 20 } * std::log10(ratio), DecibelFloor<T>::value);
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T FromDecibels(T db)
    {
        static_assert(std::is_floating_point_v<T>, "FromDecibels supports floating-point types only");
        return std::pow(T{ 10 }, db / T{ 20 });
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T AttenuationDb(T passbandRatio, T stopbandRatio)
    {
        static_assert(std::is_floating_point_v<T>, "AttenuationDb supports floating-point types only");
        return ToDecibels(passbandRatio) - ToDecibels(stopbandRatio);
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T RippleDb(T maxRatio, T minRatio)
    {
        static_assert(std::is_floating_point_v<T>, "RippleDb supports floating-point types only");
        return ToDecibels(maxRatio) - ToDecibels(minRatio);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template float ToDecibels<float>(float);
    extern template float FromDecibels<float>(float);
    extern template float AttenuationDb<float>(float, float);
    extern template float RippleDb<float>(float, float);
#endif
}

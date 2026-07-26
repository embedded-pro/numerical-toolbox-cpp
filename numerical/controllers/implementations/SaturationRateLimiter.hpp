#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <algorithm>
#include <type_traits>

namespace controllers
{
    template<typename T>
    class Saturation
    {
        static_assert(std::is_floating_point_v<T>, "Saturation supports floating-point types");

    public:
        Saturation(T lo, T hi)
            : lo{ lo }
            , hi{ hi }
        {
        }

        OPTIMIZE_FOR_SPEED T Clamp(T u) const
        {
            return std::min(std::max(u, lo), hi);
        }

    private:
        T lo;
        T hi;
    };

    template<typename T>
    class RateLimiter
    {
        static_assert(std::is_floating_point_v<T>, "RateLimiter supports floating-point types");

    public:
        RateLimiter(T maxRate, T sampleTime)
            : maxRate{ maxRate }
            , sampleTime{ sampleTime }
        {
        }

        OPTIMIZE_FOR_SPEED T Limit(T u)
        {
            if (!primed)
            {
                previous = u;
                primed = true;
                return u;
            }

            T step{ maxRate * sampleTime };
            T delta{ std::min(std::max(u - previous, -step), step) };
            previous = previous + delta;
            return previous;
        }

        void Reset(T value = T{ 0 })
        {
            previous = value;
            primed = true;
        }

    private:
        T maxRate;
        T sampleTime;
        T previous{};
        bool primed{ false };
    };

    template<typename T>
    class SlewLimitedSaturation
    {
        static_assert(std::is_floating_point_v<T>, "SlewLimitedSaturation supports floating-point types");

    public:
        SlewLimitedSaturation(T lo, T hi, T maxRate, T sampleTime)
            : clampBlock{ lo, hi }
            , slewBlock{ maxRate, sampleTime }
        {
        }

        OPTIMIZE_FOR_SPEED T Apply(T u)
        {
            return clampBlock.Clamp(slewBlock.Limit(u));
        }

    private:
        Saturation<T> clampBlock;
        RateLimiter<T> slewBlock;
    };

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Saturation<float>;
    extern template class RateLimiter<float>;
    extern template class SlewLimitedSaturation<float>;
#endif
}

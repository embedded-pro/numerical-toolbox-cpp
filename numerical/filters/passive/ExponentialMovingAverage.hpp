#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include <cassert>
#include <numbers>
#include <type_traits>

namespace filters::passive
{
    template<typename T>
    class ExponentialMovingAverage
    {
        static_assert(std::is_floating_point_v<T>, "ExponentialMovingAverage supports floating-point types");

    public:
        explicit ExponentialMovingAverage(T alpha, T initial = T{}) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T input) noexcept;
        void Reset(T value = T{}) noexcept;
        void SetAlpha(T alpha) noexcept;

        static T AlphaFromCutoff(T cutoffHz, T sampleRateHz) noexcept;

    private:
        T alpha;
        T state;
    };

    ////    Implementation    ////

    template<typename T>
    ExponentialMovingAverage<T>::ExponentialMovingAverage(T alpha, T initial) noexcept
        : alpha{ alpha }
        , state{ initial }
    {
        assert(alpha > T{} && alpha <= T{ 1 });
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T ExponentialMovingAverage<T>::Filter(T input) noexcept
    {
        state = state + alpha * (input - state);
        return state;
    }

    template<typename T>
    void ExponentialMovingAverage<T>::Reset(T value) noexcept
    {
        state = value;
    }

    template<typename T>
    void ExponentialMovingAverage<T>::SetAlpha(T newAlpha) noexcept
    {
        assert(newAlpha > T{} && newAlpha <= T{ 1 });
        alpha = newAlpha;
    }

    template<typename T>
    T ExponentialMovingAverage<T>::AlphaFromCutoff(T cutoffHz, T sampleRateHz) noexcept
    {
        const T dt{ T{ 1 } / sampleRateHz };
        const T rc{ T{ 1 } / (T{ 2 } * std::numbers::pi_v<T> * cutoffHz) };
        return dt / (rc + dt);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ExponentialMovingAverage<float>;
#endif
}

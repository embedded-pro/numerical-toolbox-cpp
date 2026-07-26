#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/RecursiveBuffer.hpp"

namespace filters::passive
{
    template<typename T, std::size_t N>
    class MovingAverage
    {
        static_assert(std::is_floating_point_v<T>, "MovingAverage supports floating-point types");

    public:
        explicit MovingAverage(T initial = T{}) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T input) noexcept;
        void Reset(T value = T{}) noexcept;

    private:
        math::RecursiveBuffer<T, N> window;
        T sum;
        T invN;

        [[no_unique_address]] math::Index n;
    };

    template<typename T, std::size_t N>
    MovingAverage<T, N>::MovingAverage(T initial) noexcept
        : sum{initial * static_cast<T>(N)}
        , invN{T{1} / static_cast<T>(N)}
    {
        for (std::size_t i = 0; i < N; ++i)
            window.Update(initial);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED T MovingAverage<T, N>::Filter(T input) noexcept
    {
        T oldest = window[n - static_cast<int32_t>(N - 1)];
        sum = sum + input - oldest;
        window.Update(input);
        return sum * invN;
    }

    template<typename T, std::size_t N>
    void MovingAverage<T, N>::Reset(T value) noexcept
    {
        window.Reset();
        sum = T{};
        if (value != T{})
        {
            for (std::size_t i = 0; i < N; ++i)
                window.Update(value);
            sum = value * static_cast<T>(N);
        }
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class MovingAverage<float, 4>;
#endif
}

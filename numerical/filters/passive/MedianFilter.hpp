#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace filters::passive
{
    template<typename T, std::size_t N>
    class MedianFilter
    {
        static_assert(std::is_floating_point_v<T>, "MedianFilter supports floating-point types");
        static_assert(N > 0, "MedianFilter window size must be greater than zero");

    public:
        explicit MedianFilter(T initial = T{}) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T input) noexcept;
        void Reset(T value = T{}) noexcept;

    private:
        std::array<T, N> window;
        std::array<T, N> scratch;
        std::size_t head;
    };

    template<typename T, std::size_t N>
    MedianFilter<T, N>::MedianFilter(T initial) noexcept
        : head{ 0 }
    {
        window.fill(initial);
        scratch.fill(T{});
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED T MedianFilter<T, N>::Filter(T input) noexcept
    {
        window[head] = input;
        head = (head + 1) % N;

        scratch = window;

        for (std::size_t i = 1; i < N; ++i)
        {
            T key = scratch[i];
            std::size_t j = i;
            while (j > 0 && scratch[j - 1] > key)
            {
                scratch[j] = scratch[j - 1];
                --j;
            }
            scratch[j] = key;
        }

        return scratch[N / 2];
    }

    template<typename T, std::size_t N>
    void MedianFilter<T, N>::Reset(T value) noexcept
    {
        window.fill(value);
        scratch.fill(T{});
        head = 0;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class MedianFilter<float, 3>;
    extern template class MedianFilter<float, 5>;
#endif
}

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace estimators
{
    template<typename T, std::size_t Taps>
    class LmsAdaptiveFilter
    {
        static_assert(std::is_floating_point_v<T>, "LmsAdaptiveFilter supports floating-point types only");
        static_assert(Taps > 0, "Taps must be greater than zero");

    public:
        struct Result
        {
            T output{};
            T error{};
        };

        explicit LmsAdaptiveFilter(T mu, bool normalized = false, T epsilon = T{ 1e-6 });

        OPTIMIZE_FOR_SPEED Result Update(T input, T desired);
        const std::array<T, Taps>& Weights() const;
        void Reset();

    private:
        std::array<T, Taps> weights{};
        std::array<T, Taps> delayLine{};
        std::size_t head{ 0 };
        T stepSize;
        bool useNlms;
        T regularizer;

        T DotProduct() const;
        T InputEnergy() const;
        T GetSample(std::size_t tapIndex) const;
    };

    template<typename T, std::size_t Taps>
    LmsAdaptiveFilter<T, Taps>::LmsAdaptiveFilter(T mu, bool normalized, T epsilon)
        : stepSize{ mu }
        , useNlms{ normalized }
        , regularizer{ epsilon }
    {
    }

    template<typename T, std::size_t Taps>
    T LmsAdaptiveFilter<T, Taps>::GetSample(std::size_t tapIndex) const
    {
        std::size_t index{ (head + Taps - 1u - tapIndex) % Taps };
        return delayLine[index];
    }

    template<typename T, std::size_t Taps>
    T LmsAdaptiveFilter<T, Taps>::DotProduct() const
    {
        T result{ T{ 0 } };
        for (std::size_t i{ 0 }; i < Taps; ++i)
            result += weights[i] * GetSample(i);
        return result;
    }

    template<typename T, std::size_t Taps>
    T LmsAdaptiveFilter<T, Taps>::InputEnergy() const
    {
        T result{ T{ 0 } };
        for (std::size_t i{ 0 }; i < Taps; ++i)
        {
            T s{ delayLine[i] };
            result += s * s;
        }
        return result;
    }

    template<typename T, std::size_t Taps>
    OPTIMIZE_FOR_SPEED typename LmsAdaptiveFilter<T, Taps>::Result LmsAdaptiveFilter<T, Taps>::Update(T input, T desired)
    {
        delayLine[head] = input;
        head = (head + 1) % Taps;

        T output{ DotProduct() };
        T error{ desired - output };

        T step{ stepSize };
        if (useNlms)
            step = stepSize / (regularizer + InputEnergy());

        T scaled{ step * error };
        for (std::size_t i{ 0 }; i < Taps; ++i)
            weights[i] += scaled * GetSample(i);

        return Result{ output, error };
    }

    template<typename T, std::size_t Taps>
    const std::array<T, Taps>& LmsAdaptiveFilter<T, Taps>::Weights() const
    {
        return weights;
    }

    template<typename T, std::size_t Taps>
    void LmsAdaptiveFilter<T, Taps>::Reset()
    {
        weights.fill(T{ 0 });
        delayLine.fill(T{ 0 });
        head = 0;
    }
}

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
namespace estimators
{
    extern template class LmsAdaptiveFilter<float, 4>;
}
#endif

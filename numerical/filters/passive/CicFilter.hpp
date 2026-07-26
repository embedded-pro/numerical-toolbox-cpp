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
    template<typename T>
    struct CicSample
    {
        T value{};
        bool valid{ false };
    };

    template<typename T, std::size_t M>
    class Comb
    {
        static_assert(std::is_floating_point_v<T>, "Comb supports floating-point types");

    public:
        OPTIMIZE_FOR_SPEED T PushPop(T input) noexcept;
        void Reset() noexcept;

    private:
        std::array<T, M> delay{};
        std::size_t head{ 0 };
    };

    template<typename T, std::size_t Stages, std::size_t R, std::size_t M>
    class CicDecimator
    {
        static_assert(std::is_floating_point_v<T>, "CicDecimator supports floating-point types");
        static_assert(Stages > 0, "Stages must be > 0");
        static_assert(R > 0, "R must be > 0");
        static_assert(M > 0, "M must be > 0");

    public:
        CicDecimator() noexcept = default;

        OPTIMIZE_FOR_SPEED CicSample<T> Filter(T input) noexcept;
        void Reset() noexcept;
        static constexpr T Gain() noexcept;

    private:
        std::array<T, Stages> integrator{};
        std::array<Comb<T, M>, Stages> comb{};
        std::size_t phase{ 0 };
    };

    // --- Comb implementation ---

    template<typename T, std::size_t M>
    OPTIMIZE_FOR_SPEED T Comb<T, M>::PushPop(T input) noexcept
    {
        T oldest = delay[head];
        delay[head] = input;
        head = (head + 1) % M;
        return oldest;
    }

    template<typename T, std::size_t M>
    void Comb<T, M>::Reset() noexcept
    {
        delay.fill(T{});
        head = 0;
    }

    // --- CicDecimator implementation ---

    template<typename T, std::size_t Stages, std::size_t R, std::size_t M>
    OPTIMIZE_FOR_SPEED CicSample<T> CicDecimator<T, Stages, R, M>::Filter(T input) noexcept
    {
        T acc{ input };
        for (std::size_t i = 0; i < Stages; ++i)
        {
            integrator[i] += acc;
            acc = integrator[i];
        }

        ++phase;
        if (phase < R)
            return CicSample<T>{};
        phase = 0;

        for (std::size_t i = 0; i < Stages; ++i)
        {
            T delayed = comb[i].PushPop(acc);
            acc = acc - delayed;
        }

        return CicSample<T>{ acc / Gain(), true };
    }

    template<typename T, std::size_t Stages, std::size_t R, std::size_t M>
    void CicDecimator<T, Stages, R, M>::Reset() noexcept
    {
        integrator.fill(T{});
        for (std::size_t i = 0; i < Stages; ++i)
            comb[i].Reset();
        phase = 0;
    }

    template<typename T, std::size_t Stages, std::size_t R, std::size_t M>
    constexpr T CicDecimator<T, Stages, R, M>::Gain() noexcept
    {
        T g{ static_cast<T>(R * M) };
        T result{ T{ 1 } };
        for (std::size_t i = 0; i < Stages; ++i)
            result *= g;
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class CicDecimator<float, 2, 4, 1>;
#endif
}

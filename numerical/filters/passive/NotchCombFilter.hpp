#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace filters::passive
{
    template<typename T>
    class NotchFilter
    {
        static_assert(std::is_floating_point_v<T>, "NotchFilter supports floating-point types");

    public:
        NotchFilter(T f0, T fs, T Q) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T x) noexcept;
        void Reset() noexcept;

    private:
        T b0{};
        T b1{};
        T b2{};
        T a1{};
        T a2{};
        T z1{};
        T z2{};
    };

    template<typename T, std::size_t D, bool Feedback = false>
    class CombFilter
    {
        static_assert(std::is_floating_point_v<T>, "CombFilter supports floating-point types");
        static_assert(D > 0, "CombFilter delay length must be greater than zero");

    public:
        explicit CombFilter(T gain) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T x) noexcept;
        void Reset() noexcept;

    private:
        T gain{};
        std::array<T, D> delayLine{};
        std::size_t writeIndex{};
    };

    ////    NotchFilter Implementation    ////

    template<typename T>
    NotchFilter<T>::NotchFilter(T f0, T fs, T Q) noexcept
    {
        const T w0{ T{ 2 } * std::numbers::pi_v<T> * f0 / fs };
        const T cw{ math::Cos(w0) };
        const T alpha{ math::Sin(w0) / (T{ 2 } * Q) };
        const T a0{ T{ 1 } + alpha };
        b0 = T{ 1 } / a0;
        b1 = (T{ -2 } * cw) / a0;
        b2 = T{ 1 } / a0;
        a1 = (T{ -2 } * cw) / a0;
        a2 = (T{ 1 } - alpha) / a0;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T NotchFilter<T>::Filter(T x) noexcept
    {
        const T y{ b0 * x + z1 };
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    template<typename T>
    void NotchFilter<T>::Reset() noexcept
    {
        z1 = T{};
        z2 = T{};
    }

    ////    CombFilter Implementation    ////

    template<typename T, std::size_t D, bool Feedback>
    CombFilter<T, D, Feedback>::CombFilter(T gain) noexcept
        : gain{ gain }
        , writeIndex{ 0 }
    {
        delayLine.fill(T{});
    }

    template<typename T, std::size_t D, bool Feedback>
    OPTIMIZE_FOR_SPEED T CombFilter<T, D, Feedback>::Filter(T x) noexcept
    {
        const T d{ delayLine[writeIndex] };
        T y{};
        if constexpr (!Feedback)
        {
            y = x - gain * d;
            delayLine[writeIndex] = x;
        }
        else
        {
            y = x + gain * d;
            delayLine[writeIndex] = y;
        }
        writeIndex = (writeIndex + 1) % D;
        return y;
    }

    template<typename T, std::size_t D, bool Feedback>
    void CombFilter<T, D, Feedback>::Reset() noexcept
    {
        delayLine.fill(T{});
        writeIndex = 0;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class NotchFilter<float>;
    extern template class CombFilter<float, 20, false>;
    extern template class CombFilter<float, 20, true>;
#endif
}

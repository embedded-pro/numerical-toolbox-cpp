#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <cassert>
#include <numbers>
#include <type_traits>

namespace filters
{
    template<typename T>
    class ComplementaryFilter
    {
        static_assert(std::is_floating_point_v<T>, "ComplementaryFilter supports floating-point types");

    public:
        explicit ComplementaryFilter(T alpha, T Ts, T initial = T{}, bool wrapAngle = false) noexcept;

        OPTIMIZE_FOR_SPEED T Update(T rate, T measuredAngle) noexcept;
        void Reset(T angle = T{}) noexcept;
        void SetAlpha(T alpha) noexcept;

        static T AlphaFromTau(T tau, T Ts) noexcept;

    private:
        T alpha;
        T Ts;
        T angle;
        bool wrapAngle;

        static T WrapToPi(T x) noexcept;
    };

    ////    Implementation    ////

    template<typename T>
    ComplementaryFilter<T>::ComplementaryFilter(T alpha, T Ts, T initial, bool wrapAngle) noexcept
        : alpha{ alpha }
        , Ts{ Ts }
        , angle{ initial }
        , wrapAngle{ wrapAngle }
    {
        assert(alpha >= T{} && alpha <= T{ 1 });
        assert(Ts > T{});
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T ComplementaryFilter<T>::Update(T rate, T measuredAngle) noexcept
    {
        T predicted{ angle + rate * Ts };
        if (wrapAngle)
        {
            T delta{ WrapToPi(measuredAngle - predicted) };
            angle = WrapToPi(predicted + (T{ 1 } - alpha) * delta);
        }
        else
        {
            angle = alpha * predicted + (T{ 1 } - alpha) * measuredAngle;
        }
        return angle;
    }

    template<typename T>
    void ComplementaryFilter<T>::Reset(T value) noexcept
    {
        angle = value;
    }

    template<typename T>
    void ComplementaryFilter<T>::SetAlpha(T newAlpha) noexcept
    {
        assert(newAlpha >= T{} && newAlpha <= T{ 1 });
        alpha = newAlpha;
    }

    template<typename T>
    T ComplementaryFilter<T>::AlphaFromTau(T tau, T Ts) noexcept
    {
        return tau / (tau + Ts);
    }

    template<typename T>
    T ComplementaryFilter<T>::WrapToPi(T x) noexcept
    {
        constexpr T pi{ std::numbers::pi_v<T> };
        constexpr T twoPi{ T{ 2 } * std::numbers::pi_v<T> };
        T wrapped{ math::Fmod(x + pi, twoPi) };
        if (wrapped < T{})
            wrapped += twoPi;
        return wrapped - pi;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ComplementaryFilter<float>;
#endif
}

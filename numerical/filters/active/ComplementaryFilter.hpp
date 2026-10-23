#pragma once

#include "numerical/math/AngleUtilities.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <cassert>
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
        OPTIMIZE_FOR_SPEED T Update(T rate, T measuredAngle, T measuredTs) noexcept;
        void Reset(T angle = T{}) noexcept;
        void SetAlpha(T alpha) noexcept;

        static T AlphaFromTau(T tau, T Ts) noexcept;

    private:
        T alpha;
        T Ts;
        T angle;
        bool wrapAngle;
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
        return Update(rate, measuredAngle, Ts);
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T ComplementaryFilter<T>::Update(T rate, T measuredAngle, T measuredTs) noexcept
    {
        assert(measuredTs > T{});

        T predicted{ angle + rate * measuredTs };
        if (wrapAngle)
        {
            T delta{ math::AngleDifference(measuredAngle, predicted) };
            angle = math::WrapToPi(predicted + (T{ 1 } - alpha) * delta);
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

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ComplementaryFilter<float>;
#endif
}

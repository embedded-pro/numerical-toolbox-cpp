#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/motion_profiles/MotionProfileTypes.hpp"
#include <algorithm>
#include <type_traits>

namespace motion_profiles
{
    template<typename T>
    struct AsymmetricMotionLimits
    {
        static_assert(std::is_floating_point_v<T>, "AsymmetricMotionLimits supports floating-point types only");

        T vMax{};
        T aMax{};
        T dMax{};
    };

    template<typename T>
    class TrapezoidalProfileFromState
    {
        static_assert(std::is_floating_point_v<T>, "TrapezoidalProfileFromState supports floating-point types only");

    public:
        TrapezoidalProfileFromState(T q0, T v0, T qf, const AsymmetricMotionLimits<T>& limits);

        OPTIMIZE_FOR_SPEED TrajectoryState<T> Sample(T t) const;
        T Duration() const;

    private:
        void PlanShortMove(T distance, T aMax, T dMax);

        T q0;
        T v0;
        T qf;
        T direction{ T{ 1 } };
        T ramp{};
        T cruise{};
        T deceleration{};
        T tRamp{};
        T tCruise{};
        T tf{};
    };

    template<typename T>
    TrapezoidalProfileFromState<T>::TrapezoidalProfileFromState(T q0, T v0, T qf, const AsymmetricMotionLimits<T>& limits)
        : q0{ q0 }
        , v0{ v0 }
        , qf{ qf }
    {
        really_assert(limits.vMax > T{});
        really_assert(limits.aMax > T{});
        really_assert(limits.dMax > T{});

        const T distance{ qf - q0 };
        const T stoppingDistance{ v0 * math::Abs(v0) / (T{ 2 } * limits.dMax) };
        direction = distance - stoppingDistance >= T{} ? T{ 1 } : T{ -1 };
        deceleration = direction * limits.dMax;
        cruise = direction * limits.vMax;
        ramp = direction * v0 > limits.vMax ? -deceleration : direction * limits.aMax;

        tRamp = (cruise - v0) / ramp;
        const T tDecelerate{ limits.vMax / limits.dMax };
        const T minimumDistance{ T{ 0.5 } * tRamp * (cruise + v0) + T{ 0.5 } * tDecelerate * cruise };

        if (direction * distance >= direction * minimumDistance)
            tCruise = (distance - minimumDistance) / cruise;
        else
            PlanShortMove(distance, limits.aMax, limits.dMax);

        tf = tRamp + tCruise + math::Abs(cruise) / limits.dMax;
    }

    template<typename T>
    void TrapezoidalProfileFromState<T>::PlanShortMove(T distance, T aMax, T dMax)
    {
        const T squared{ (T{ 2 } * direction * distance * aMax * dMax + v0 * v0 * dMax) / (aMax + dMax) };
        cruise = direction * math::Sqrt(std::max(squared, T{}));
        tRamp = std::max((cruise - v0) / ramp, T{});
        tCruise = T{};
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED TrajectoryState<T> TrapezoidalProfileFromState<T>::Sample(T t) const
    {
        t = std::max(t, T{});
        if (t >= tf)
            return { qf, T{}, T{}, T{} };

        if (t < tRamp)
            return { q0 + v0 * t + T{ 0.5 } * ramp * t * t, v0 + ramp * t, ramp, T{} };

        if (t < tRamp + tCruise)
            return { q0 + T{ 0.5 } * tRamp * (cruise + v0) + cruise * (t - tRamp), cruise, T{}, T{} };

        const T remaining{ tf - t };
        return { qf - T{ 0.5 } * deceleration * remaining * remaining, deceleration * remaining, -deceleration, T{} };
    }

    template<typename T>
    T TrapezoidalProfileFromState<T>::Duration() const
    {
        return tf;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class TrapezoidalProfileFromState<float>;
#endif
}

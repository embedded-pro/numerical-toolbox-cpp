#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/motion_profiles/MotionProfileTypes.hpp"
#include <algorithm>
#include <optional>
#include <type_traits>

namespace motion_profiles
{
    template<typename T>
    class TrapezoidalProfile
    {
        static_assert(std::is_floating_point_v<T>, "TrapezoidalProfile supports floating-point types only");

    public:
        TrapezoidalProfile(T q0, T qf, const MotionLimits<T>& limits);
        static std::optional<TrapezoidalProfile> PlanWithDuration(T q0, T qf, T aMax, T duration);

        OPTIMIZE_FOR_SPEED TrajectoryState<T> Sample(T t) const;
        T Duration() const;
        bool IsTriangular() const;

    private:
        TrapezoidalProfile(T q0, T qf, T aMax);

        T q0;
        T direction;
        T distance;
        T aMax;
        T vPeak{};
        T tAccel{};
        T tCruise{};
        T tf{};
    };

    template<typename T>
    TrapezoidalProfile<T>::TrapezoidalProfile(T q0, T qf, T aMax)
        : q0{ q0 }
        , direction{ qf >= q0 ? T{ 1 } : T{ -1 } }
        , distance{ math::Abs(qf - q0) }
        , aMax{ aMax }
    {}

    template<typename T>
    TrapezoidalProfile<T>::TrapezoidalProfile(T q0, T qf, const MotionLimits<T>& limits)
        : TrapezoidalProfile{ q0, qf, limits.aMax }
    {
        really_assert(limits.vMax > T{});
        really_assert(limits.aMax > T{});

        const T blendDistance{ limits.vMax * limits.vMax / limits.aMax };
        if (distance >= blendDistance)
        {
            vPeak = limits.vMax;
            tCruise = (distance - blendDistance) / vPeak;
        }
        else
            vPeak = math::Sqrt(distance * aMax);

        tAccel = vPeak / aMax;
        tf = T{ 2 } * tAccel + tCruise;
    }

    template<typename T>
    std::optional<TrapezoidalProfile<T>> TrapezoidalProfile<T>::PlanWithDuration(T q0, T qf, T aMax, T duration)
    {
        TrapezoidalProfile profile{ q0, qf, aMax };
        const T discriminant{ aMax * aMax * duration * duration - T{ 4 } * aMax * profile.distance };
        if (aMax <= T{} || duration <= T{} || discriminant < T{})
            return std::nullopt;

        profile.vPeak = T{ 2 } * aMax * profile.distance / (aMax * duration + math::Sqrt(discriminant));
        profile.tAccel = profile.vPeak / aMax;
        profile.tCruise = std::max(duration - T{ 2 } * profile.tAccel, T{});
        profile.tf = duration;
        return profile;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED TrajectoryState<T> TrapezoidalProfile<T>::Sample(T t) const
    {
        t = std::max(t, T{});
        if (t >= tf)
            return { q0 + direction * distance, T{}, T{}, T{} };

        if (t < tAccel)
            return { q0 + direction * T{ 0.5 } * aMax * t * t, direction * aMax * t, direction * aMax, T{} };

        if (t < tAccel + tCruise)
            return { q0 + direction * vPeak * (t - T{ 0.5 } * tAccel), direction * vPeak, T{}, T{} };

        const T remaining{ tf - t };
        return { q0 + direction * (distance - T{ 0.5 } * aMax * remaining * remaining), direction * aMax * remaining, -direction * aMax, T{} };
    }

    template<typename T>
    T TrapezoidalProfile<T>::Duration() const
    {
        return tf;
    }

    template<typename T>
    bool TrapezoidalProfile<T>::IsTriangular() const
    {
        return tCruise <= T{};
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class TrapezoidalProfile<float>;
#endif
}

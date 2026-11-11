#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/motion_profiles/MotionProfileTypes.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

namespace motion_profiles
{
    template<typename T>
    struct JerkLimitedMotionLimits
    {
        static_assert(std::is_floating_point_v<T>, "JerkLimitedMotionLimits supports floating-point types only");

        T vMax{};
        T aMax{};
        T dMax{};
        T jMax{};
    };

    template<typename T>
    class SCurveProfileFromState
    {
        static_assert(std::is_floating_point_v<T>, "SCurveProfileFromState supports floating-point types only");

    public:
        SCurveProfileFromState(T q0, T v0, T a0, T qf, const JerkLimitedMotionLimits<T>& limits);

        OPTIMIZE_FOR_SPEED TrajectoryState<T> Sample(T t) const;
        T Duration() const;

    private:
        static constexpr std::size_t segments{ 7 };
        static constexpr std::size_t bisectionIterations{ 32 };

        struct Kinematics
        {
            T position;
            T velocity;
            T acceleration;
        };

        struct Ramp
        {
            T rise;
            T hold;
            T fall;
            T riseJerk;
            T fallJerk;
        };

        Ramp PlanRamp(T vStart, T aStart, T vTarget) const;
        T RampLimit(T sign, T vStart, T vTarget) const;
        static Kinematics Advance(const Kinematics& state, T duration, T jerk);
        static Kinematics Traverse(const Kinematics& state, const Ramp& ramp);
        T DistanceVia(T peak) const;
        T PeakVelocity(T distance) const;
        T Bisect(T low, T high, T distance) const;
        void Build(T peak, T cruise);
        OPTIMIZE_FOR_SPEED std::size_t SegmentAt(T t) const;

        T q0;
        T v0;
        T a0;
        T qf;
        JerkLimitedMotionLimits<T> limits;
        T direction{ T{ 1 } };
        std::array<T, segments + 1> segStart{};
        std::array<T, segments> segPos{};
        std::array<T, segments> segVel{};
        std::array<T, segments> segAcc{};
        std::array<T, segments> segJerk{};
        T tf{};
    };

    template<typename T>
    SCurveProfileFromState<T>::SCurveProfileFromState(T q0, T v0, T a0, T qf, const JerkLimitedMotionLimits<T>& limits)
        : q0{ q0 }
        , v0{ v0 }
        , a0{ a0 }
        , qf{ qf }
        , limits{ limits }
    {
        really_assert(limits.vMax > T{});
        really_assert(limits.aMax > T{});
        really_assert(limits.dMax > T{});
        really_assert(limits.jMax > T{});
        really_assert(math::Abs(a0) <= std::max(limits.aMax, limits.dMax));

        const T distance{ qf - q0 };
        const T stoppingDistance{ Traverse({ T{}, v0, a0 }, PlanRamp(v0, a0, T{})).position };
        direction = distance - stoppingDistance >= T{} ? T{ 1 } : T{ -1 };

        const T frameDistance{ direction * distance };
        const T peak{ PeakVelocity(frameDistance) };
        const T residual{ std::max(frameDistance - DistanceVia(peak), T{}) };
        Build(peak, peak > T{} ? residual / peak : T{});
    }

    template<typename T>
    typename SCurveProfileFromState<T>::Ramp SCurveProfileFromState<T>::PlanRamp(T vStart, T aStart, T vTarget) const
    {
        const T jMax{ limits.jMax };
        const T vZero{ vStart + aStart * math::Abs(aStart) / (T{ 2 } * jMax) };
        const T sign{ vTarget >= vZero ? T{ 1 } : T{ -1 } };
        const T freePeak{ math::Sqrt(std::max((T{ 2 } * jMax * sign * (vTarget - vStart) + aStart * aStart) / T{ 2 }, T{})) };
        const T peak{ std::min(freePeak, RampLimit(sign, vStart, vTarget)) };

        const T rise{ math::Abs(sign * peak - aStart) / jMax };
        const T fall{ peak / jMax };
        const T reached{ vStart + T{ 0.5 } * (aStart + sign * peak) * rise + T{ 0.5 } * sign * peak * fall };
        const T hold{ peak > T{} ? std::max((vTarget - reached) / (sign * peak), T{}) : T{} };

        return { rise, hold, fall, sign * peak >= aStart ? jMax : -jMax, -sign * jMax };
    }

    template<typename T>
    T SCurveProfileFromState<T>::RampLimit(T sign, T vStart, T vTarget) const
    {
        if (sign * vStart <= T{} && sign * vTarget <= T{})
            return limits.dMax;

        if (sign * vStart >= T{} && sign * vTarget >= T{})
            return limits.aMax;

        return std::min(limits.aMax, limits.dMax);
    }

    template<typename T>
    typename SCurveProfileFromState<T>::Kinematics SCurveProfileFromState<T>::Advance(const Kinematics& state, T duration, T jerk)
    {
        return {
            state.position + duration * (state.velocity + duration * (state.acceleration / T{ 2 } + duration * jerk / T{ 6 })),
            state.velocity + duration * (state.acceleration + duration * jerk / T{ 2 }),
            state.acceleration + duration * jerk
        };
    }

    template<typename T>
    typename SCurveProfileFromState<T>::Kinematics SCurveProfileFromState<T>::Traverse(const Kinematics& state, const Ramp& ramp)
    {
        return Advance(Advance(Advance(state, ramp.rise, ramp.riseJerk), ramp.hold, T{}), ramp.fall, ramp.fallJerk);
    }

    template<typename T>
    T SCurveProfileFromState<T>::DistanceVia(T peak) const
    {
        const T vStart{ direction * v0 };
        const T aStart{ direction * a0 };
        const T toPeak{ Traverse({ T{}, vStart, aStart }, PlanRamp(vStart, aStart, peak)).position };
        return toPeak + Traverse({ T{}, peak, T{} }, PlanRamp(peak, T{}, T{})).position;
    }

    template<typename T>
    T SCurveProfileFromState<T>::PeakVelocity(T distance) const
    {
        if (DistanceVia(limits.vMax) <= distance)
            return limits.vMax;

        const T aStart{ direction * a0 };
        const T vZero{ direction * v0 + aStart * math::Abs(aStart) / (T{ 2 } * limits.jMax) };
        const T split{ std::clamp(vZero, T{}, limits.vMax) };

        if (DistanceVia(split) <= distance)
            return Bisect(split, limits.vMax, distance);

        return Bisect(T{}, split, distance);
    }

    template<typename T>
    T SCurveProfileFromState<T>::Bisect(T low, T high, T distance) const
    {
        for (std::size_t i = 0; i < bisectionIterations; ++i)
        {
            const T middle{ T{ 0.5 } * (low + high) };
            if (DistanceVia(middle) <= distance)
                low = middle;
            else
                high = middle;
        }

        return low;
    }

    template<typename T>
    void SCurveProfileFromState<T>::Build(T peak, T cruise)
    {
        const Ramp up{ PlanRamp(direction * v0, direction * a0, peak) };
        const Ramp down{ PlanRamp(peak, T{}, T{}) };
        const std::array<T, segments> durations{ up.rise, up.hold, up.fall, cruise, down.rise, down.hold, down.fall };
        const std::array<T, segments> jerks{ up.riseJerk, T{}, up.fallJerk, T{}, down.riseJerk, T{}, down.fallJerk };

        Kinematics state{ q0, v0, a0 };
        for (std::size_t i = 0; i < segments; ++i)
        {
            segPos[i] = state.position;
            segVel[i] = state.velocity;
            segAcc[i] = state.acceleration;
            segJerk[i] = direction * jerks[i];
            segStart[i + 1] = segStart[i] + durations[i];
            state = Advance(state, durations[i], segJerk[i]);
        }

        tf = segStart[segments];
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED std::size_t SCurveProfileFromState<T>::SegmentAt(T t) const
    {
        if (t < segStart[4])
            return t < segStart[2] ? (t < segStart[1] ? 0 : 1) : (t < segStart[3] ? 2 : 3);

        return t < segStart[6] ? (t < segStart[5] ? 4 : 5) : 6;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED TrajectoryState<T> SCurveProfileFromState<T>::Sample(T t) const
    {
        t = std::max(t, T{});
        if (t >= tf)
            return { qf, T{}, T{}, T{} };

        const std::size_t i{ SegmentAt(t) };
        const T tau{ t - segStart[i] };
        const T jerk{ segJerk[i] };
        return {
            segPos[i] + tau * (segVel[i] + tau * (segAcc[i] / T{ 2 } + tau * jerk / T{ 6 })),
            segVel[i] + tau * (segAcc[i] + tau * jerk / T{ 2 }),
            segAcc[i] + tau * jerk,
            jerk
        };
    }

    template<typename T>
    T SCurveProfileFromState<T>::Duration() const
    {
        return tf;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SCurveProfileFromState<float>;
#endif
}

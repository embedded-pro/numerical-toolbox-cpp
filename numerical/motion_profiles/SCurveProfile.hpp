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
    class SCurveProfile
    {
        static_assert(std::is_floating_point_v<T>, "SCurveProfile supports floating-point types only");

    public:
        SCurveProfile(T q0, T qf, const MotionLimits<T>& limits);

        OPTIMIZE_FOR_SPEED TrajectoryState<T> Sample(T t) const;
        T Duration() const;
        bool ReachesMaxAccel() const;
        bool ReachesMaxVel() const;

    private:
        static constexpr std::size_t phases{ 7 };
        static constexpr std::array<T, phases> jerkSign{ T{ 1 }, T{}, T{ -1 }, T{}, T{ -1 }, T{}, T{ 1 } };

        void Plan(const MotionLimits<T>& limits);
        void Integrate(T jerkPhase, T accelPhase, T cruisePhase);

        T q0;
        T direction;
        T distance;
        T jMax;
        std::array<T, phases + 1> segStart{};
        std::array<T, phases + 1> segPos{};
        std::array<T, phases + 1> segVel{};
        std::array<T, phases + 1> segAcc{};
        T tf{};
        bool reachesMaxAccel{ false };
        bool reachesMaxVel{ false };
    };

    template<typename T>
    SCurveProfile<T>::SCurveProfile(T q0, T qf, const MotionLimits<T>& limits)
        : q0{ q0 }
        , direction{ qf >= q0 ? T{ 1 } : T{ -1 } }
        , distance{ math::Abs(qf - q0) }
        , jMax{ limits.jMax }
    {
        really_assert(limits.vMax > T{});
        really_assert(limits.aMax > T{});
        really_assert(limits.jMax > T{});

        if (distance > T{})
            Plan(limits);
    }

    template<typename T>
    void SCurveProfile<T>::Plan(const MotionLimits<T>& limits)
    {
        const T vMax{ limits.vMax };
        const T aMax{ limits.aMax };

        T tj{ aMax / jMax };
        T ta{ tj + vMax / aMax };
        if (vMax * jMax < aMax * aMax)
        {
            tj = math::Sqrt(vMax / jMax);
            ta = T{ 2 } * tj;
        }

        T tv{ distance / vMax - ta };
        if (tv < T{})
        {
            tv = T{};
            tj = aMax / jMax;
            const T delta{ aMax * aMax * aMax * aMax / (jMax * jMax) + T{ 4 } * aMax * distance };
            ta = (aMax * aMax / jMax + math::Sqrt(delta)) / (T{ 2 } * aMax);

            if (ta < T{ 2 } * tj)
            {
                tj = math::Cbrt(distance / (T{ 2 } * jMax));
                ta = T{ 2 } * tj;
            }
        }

        const T accelPhase{ std::max(ta - T{ 2 } * tj, T{}) };
        reachesMaxAccel = accelPhase > T{};
        reachesMaxVel = tv > T{};
        Integrate(tj, accelPhase, tv);
    }

    template<typename T>
    void SCurveProfile<T>::Integrate(T jerkPhase, T accelPhase, T cruisePhase)
    {
        const std::array<T, phases> duration{ jerkPhase, accelPhase, jerkPhase, cruisePhase, jerkPhase, accelPhase, jerkPhase };

        for (std::size_t i = 0; i < phases; ++i)
        {
            const T tau{ duration[i] };
            const T jerk{ jerkSign[i] * jMax };
            segPos[i + 1] = segPos[i] + tau * (segVel[i] + tau * (segAcc[i] / T{ 2 } + tau * jerk / T{ 6 }));
            segVel[i + 1] = segVel[i] + tau * (segAcc[i] + tau * jerk / T{ 2 });
            segAcc[i + 1] = segAcc[i] + tau * jerk;
            segStart[i + 1] = segStart[i] + tau;
        }

        tf = segStart[phases];
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED TrajectoryState<T> SCurveProfile<T>::Sample(T t) const
    {
        t = std::max(t, T{});
        if (t >= tf)
            return { q0 + direction * distance, T{}, T{}, T{} };

        std::size_t i{ 0 };
        while (i + 1 < phases && segStart[i + 1] <= t)
            ++i;

        const T tau{ t - segStart[i] };
        const T jerk{ jerkSign[i] * jMax };
        return {
            q0 + direction * (segPos[i] + tau * (segVel[i] + tau * (segAcc[i] / T{ 2 } + tau * jerk / T{ 6 }))),
            direction * (segVel[i] + tau * (segAcc[i] + tau * jerk / T{ 2 })),
            direction * (segAcc[i] + tau * jerk),
            direction * jerk
        };
    }

    template<typename T>
    T SCurveProfile<T>::Duration() const
    {
        return tf;
    }

    template<typename T>
    bool SCurveProfile<T>::ReachesMaxAccel() const
    {
        return reachesMaxAccel;
    }

    template<typename T>
    bool SCurveProfile<T>::ReachesMaxVel() const
    {
        return reachesMaxVel;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SCurveProfile<float>;
#endif
}

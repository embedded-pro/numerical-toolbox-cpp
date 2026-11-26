#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <type_traits>

namespace controllers
{
    template<typename T>
    class PidContinuous
    {
        static_assert(std::is_floating_point_v<T>, "PidContinuous supports floating-point types");

    public:
        struct Gains
        {
            T kp{};
            T ki{};
            T kd{};
        };

        PidContinuous(T minimum, T maximum);

        void Reset();
        OPTIMIZE_FOR_SPEED T Update(T setpoint, T measured, const Gains& gains, T interval);
        T Output() const;

    private:
        T minimum;
        T maximum;
        T output{};
        T previousError{};
        T previousDerivative{};
        bool started{ false };
    };

    template<typename T>
    PidContinuous<T>::PidContinuous(T minimum, T maximum)
        : minimum{ minimum }
        , maximum{ maximum }
    {
        really_assert(minimum < maximum);
    }

    template<typename T>
    void PidContinuous<T>::Reset()
    {
        output = T{};
        previousError = T{};
        previousDerivative = T{};
        started = false;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T PidContinuous<T>::Update(T setpoint, T measured, const Gains& gains, T interval)
    {
        if (!(interval > T{}) || !math::IsFinite(interval))
            return output;

        const T error{ setpoint - measured };
        const T derivative{ started ? (error - previousError) / interval : T{} };
        const T increment{ gains.kp * (error - previousError) + gains.ki * interval * error + gains.kd * (derivative - previousDerivative) };

        if (!math::IsFinite(increment))
            return output;

        output = std::clamp(output + increment, minimum, maximum);
        previousError = error;
        previousDerivative = derivative;
        started = true;
        return output;
    }

    template<typename T>
    T PidContinuous<T>::Output() const
    {
        return output;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PidContinuous<float>;
#endif
}

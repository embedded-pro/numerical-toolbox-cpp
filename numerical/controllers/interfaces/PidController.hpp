#pragma once

#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename QNumberType>
    class PidController
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "Pid can only be instantiated with math::QNumber types.");

    public:
        struct Tunings
        {
            QNumberType kp;
            QNumberType ki;
            QNumberType kd;
        };

        struct Limits
        {
            QNumberType min;
            QNumberType max;
        };

        virtual void SetTunings(Tunings tunings) = 0;
        virtual void SetLimits(Limits limits) = 0;
        virtual void SetPoint(QNumberType setPoint) = 0;
        virtual void Enable() = 0;
        virtual void Disable() = 0;
    };
}

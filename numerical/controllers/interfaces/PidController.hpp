#pragma once

#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename QNumberType>
    struct PidTunings
    {
        QNumberType kp;
        QNumberType ki;
        QNumberType kd;
    };

    template<typename QNumberType>
    struct PidLimits
    {
        QNumberType min;
        QNumberType max;
    };

    template<typename QNumberType>
    class AsynchronousPidController
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "Pid can only be instantiated with math::QNumber types.");

    public:
        virtual void SetTunings(PidTunings<QNumberType> tunings) = 0;
        virtual void SetLimits(PidLimits<QNumberType> limits) = 0;
        virtual void SetPoint(QNumberType setPoint) = 0;
        virtual void Enable() = 0;
        virtual void Disable() = 0;
    };

    template<typename QNumberType>
    class SynchronousPidController
        : public AsynchronousPidController<QNumberType>
    {
    public:
        virtual QNumberType Process(QNumberType processVariable) = 0;
    };
}

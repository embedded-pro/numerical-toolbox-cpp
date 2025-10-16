#ifndef CONTROLLERS_PID_HPP
#define CONTROLLERS_PID_HPP

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include <optional>

namespace controllers
{
    template<typename QNumberType>
    class Pid
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Pid can only be instantiated with math::QNumber types.");

    public:
        struct Tunnings
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

        Pid(Tunnings tunnings, Limits limits, bool autoMode = true);

        void SetPoint(QNumberType setPoint);
        QNumberType Process(QNumberType measuredProcessVariable);
        void Enable();
        void Disable();
        void SetLimits(Limits limits);
        void SetTunnings(Tunnings tunnings);
        void Reset();

    private:
        QNumberType Clamp(QNumberType input);
        void UpdateLasts(QNumberType& controllerOutput, QNumberType& error, QNumberType& measuredProcessVariable);

    private:
        Limits limits;
        bool autoMode;

        std::optional<QNumberType> setPoint;
        QNumberType a0 = 0;
        QNumberType a1 = 0;
        QNumberType a2 = 0;

        math::Index n;
        math::RecursiveBuffer<QNumberType, 2> y;
        math::RecursiveBuffer<QNumberType, 3> x;
    };

    ////    Implementation    ////

    template<typename QNumberType>
    Pid<QNumberType>::Pid(Tunnings tunnings, Limits limits, bool autoMode)
        : limits(limits)
        , autoMode(autoMode)
    {
        really_assert(limits.min < limits.max);

        a0 = tunnings.kp + tunnings.ki + tunnings.kd;
        a1 = -tunnings.kp - (tunnings.kd + tunnings.kd);
        a2 = tunnings.kd;
    }

    template<class QNumberType>
    void Pid<QNumberType>::SetPoint(QNumberType setPoint)
    {
        this->setPoint.emplace(setPoint);
    }

    template<class QNumberType>
    void Pid<QNumberType>::Enable()
    {
        if (autoMode)
            Reset();

        autoMode = true;
    }

    template<typename QNumberType>
    void Pid<QNumberType>::Disable()
    {
        autoMode = false;
    }

    template<class QNumberType>
    void Pid<QNumberType>::SetLimits(Limits limits)
    {
        if (limits.max >= limits.min)
            std::abort();

        this->limits = limits;
    }

    template<class QNumberType>
    void Pid<QNumberType>::SetTunnings(Tunnings tunnings)
    {
        a0 = tunnings.kp + tunnings.ki + tunnings.kd;
        a1 = -tunnings.kp - (tunnings.kd + tunnings.kd);
        a2 = tunnings.kd;
    }

    template<class QNumberType>
    ALWAYS_INLINE
        QNumberType
        Pid<QNumberType>::Clamp(QNumberType input)
    {
        if (input > limits.max)
            return limits.max;

        if (input < limits.min)
            return limits.min;

        return input;
    }

    template<class QNumberType>
    ALWAYS_INLINE_HOT
        QNumberType
        Pid<QNumberType>::Process(QNumberType measuredProcessVariable)
    {
        if (!setPoint || !autoMode)
            return measuredProcessVariable;

        x.Update(*setPoint - measuredProcessVariable);
        auto output = Clamp(y[n - 1] + a0 * x[n - 0] + a1 * x[n - 1] + a2 * x[n - 2]);
        y.Update(output);

        return output;
    }

    template<class QNumberType>
    void Pid<QNumberType>::Reset()
    {
    }
}

#endif

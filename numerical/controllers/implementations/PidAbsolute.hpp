#pragma once

#include "numerical/controllers/interfaces/PidController.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <optional>

namespace controllers
{
    template<typename QNumberType>
    class PidAbsoluteBase
    {
    public:
        void SetPoint(QNumberType setPoint);
        void SetLimits(PidLimits<QNumberType> limits);
        void SetIntegralLimit(QNumberType integralLimit);
        void SetTunings(PidTunings<QNumberType> tunnings);
        void Enable();
        QNumberType Process(QNumberType processVariable);

    protected:
        PidAbsoluteBase(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits, std::optional<QNumberType> integralLimit = std::nullopt);

    private:
        QNumberType Clamp(QNumberType input, QNumberType min, QNumberType max);

    private:
        std::optional<QNumberType> setPoint;
        PidLimits<QNumberType> limits;
        QNumberType integralLimit;

        QNumberType kp;
        QNumberType ki;
        QNumberType kd;

        QNumberType integral = QNumberType(0.0f);
        QNumberType previousError = QNumberType(0.0f);
    };

    template<typename QNumberType>
    class PidAbsoluteSynchronous
        : public SynchronousPidController<QNumberType>
        , private PidAbsoluteBase<QNumberType>
    {
    public:
        PidAbsoluteSynchronous(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits);
        PidAbsoluteSynchronous(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits, QNumberType integralLimit);

        // Implementation of SynchronousPidController
        void SetPoint(QNumberType setPoint) override;
        void SetLimits(PidLimits<QNumberType> limits) override;
        void SetTunings(PidTunings<QNumberType> tunnings) override;
        void Enable() override;
        void Disable() override;
        QNumberType Process(QNumberType processVariable) override;

        void SetIntegralLimit(QNumberType integralLimit);
    };

    ////    Implementation    ////

    template<typename QNumberType>
    PidAbsoluteBase<QNumberType>::PidAbsoluteBase(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits, std::optional<QNumberType> integralLimit)
        : limits{ limits }
        , integralLimit{ integralLimit.value_or(limits.max) }
        , kp{ tunnings.kp }
        , ki{ tunnings.ki }
        , kd{ tunnings.kd }
    {
        really_assert(limits.min < limits.max);
    }

    template<class QNumberType>
    void PidAbsoluteBase<QNumberType>::SetPoint(QNumberType _setPoint)
    {
        this->setPoint.emplace(_setPoint);
    }

    template<class QNumberType>
    void PidAbsoluteBase<QNumberType>::Enable()
    {
        integral = QNumberType(0.0f);
        previousError = QNumberType(0.0f);
    }

    template<class QNumberType>
    void PidAbsoluteBase<QNumberType>::SetLimits(PidLimits<QNumberType> _limits)
    {
        really_assert(_limits.max > _limits.min);
        this->limits = _limits;
    }

    template<class QNumberType>
    void PidAbsoluteBase<QNumberType>::SetIntegralLimit(QNumberType _integralLimit)
    {
        this->integralLimit = _integralLimit;
    }

    template<class QNumberType>
    void PidAbsoluteBase<QNumberType>::SetTunings(PidTunings<QNumberType> tunings)
    {
        kp = tunings.kp;
        ki = tunings.ki;
        kd = tunings.kd;
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidAbsoluteBase<QNumberType>::Clamp(QNumberType input, QNumberType min, QNumberType max)
    {
        if (input > max)
            return max;

        if (input < min)
            return min;

        return input;
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidAbsoluteBase<QNumberType>::Process(QNumberType processVariable)
    {
        if (!setPoint.has_value())
            return processVariable;

        QNumberType error = *setPoint - processVariable;

        QNumberType P = kp * error;

        integral = integral + error;
        integral = Clamp(integral, -integralLimit, integralLimit);
        QNumberType I = ki * integral;

        QNumberType derivative = error - previousError;
        QNumberType D = kd * derivative;

        QNumberType output = P + I + D;

        output = Clamp(output, limits.min, limits.max);

        previousError = error;

        return output;
    }

    template<class QNumberType>
    PidAbsoluteSynchronous<QNumberType>::PidAbsoluteSynchronous(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits)
        : PidAbsoluteBase<QNumberType>{ tunnings, limits }
    {
    }

    template<class QNumberType>
    PidAbsoluteSynchronous<QNumberType>::PidAbsoluteSynchronous(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits, QNumberType integralLimit)
        : PidAbsoluteBase<QNumberType>{ tunnings, limits, integralLimit }
    {
    }

    template<class QNumberType>
    void PidAbsoluteSynchronous<QNumberType>::SetPoint(QNumberType setPoint)
    {
        PidAbsoluteBase<QNumberType>::SetPoint(setPoint);
    }

    template<class QNumberType>
    void PidAbsoluteSynchronous<QNumberType>::SetLimits(PidLimits<QNumberType> limits)
    {
        PidAbsoluteBase<QNumberType>::SetLimits(limits);
    }

    template<class QNumberType>
    void PidAbsoluteSynchronous<QNumberType>::SetIntegralLimit(QNumberType integralLimit)
    {
        PidAbsoluteBase<QNumberType>::SetIntegralLimit(integralLimit);
    }

    template<class QNumberType>
    void PidAbsoluteSynchronous<QNumberType>::SetTunings(PidTunings<QNumberType> tunnings)
    {
        PidAbsoluteBase<QNumberType>::SetTunings(tunnings);
    }

    template<class QNumberType>
    void PidAbsoluteSynchronous<QNumberType>::Enable()
    {
        PidAbsoluteBase<QNumberType>::Enable();
    }

    template<class QNumberType>
    void PidAbsoluteSynchronous<QNumberType>::Disable()
    {
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidAbsoluteSynchronous<QNumberType>::Process(QNumberType processVariable)
    {
        return PidAbsoluteBase<QNumberType>::Process(processVariable);
    }
}

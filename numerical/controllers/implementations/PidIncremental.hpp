#pragma once

#include "numerical/controllers/interfaces/PidController.hpp"
#include "numerical/controllers/interfaces/PidDriver.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <optional>

namespace controllers
{

    template<typename QNumberType>
    class PidIncremental
        : public PidController<QNumberType>
    {
    public:
        using PidBase = PidController<QNumberType>;

        PidIncremental(PidDriver<QNumberType>& driver, std::chrono::system_clock::duration sampleTime, typename PidBase::Tunings tunnings, typename PidBase::Limits limits);

        // Implementation of PidController
        void SetPoint(QNumberType setPoint) override;
        void SetLimits(PidBase::Limits limits) override;
        void SetTunings(PidBase::Tunings tunnings) override;
        void Enable() override;
        void Disable() override;

    private:
        QNumberType Clamp(QNumberType input);
        void UpdateLasts(QNumberType& controllerOutput, QNumberType& error, QNumberType& measuredProcessVariable);
        QNumberType Process(QNumberType measuredProcessVariable);

    private:
        PidDriver<QNumberType>& driver;
        std::chrono::system_clock::duration sampleTime;
        PidBase::Limits limits;

        std::optional<QNumberType> setPoint;
        QNumberType a0 = 0;
        QNumberType a1 = 0;
        QNumberType a2 = 0;

        QNumberType u = QNumberType(0.0f);
        QNumberType u_1 = QNumberType(0.0f);

        QNumberType e = QNumberType(0.0f);
        QNumberType e_1 = QNumberType(0.0f);
        QNumberType e_2 = QNumberType(0.0f);
    };

    ////    Implementation    ////

    template<typename QNumberType>
    PidIncremental<QNumberType>::PidIncremental(PidDriver<QNumberType>& driver, std::chrono::system_clock::duration sampleTime, typename PidBase::Tunings tunnings, typename PidBase::Limits limits)
        : driver{ driver }
        , limits{ limits }
        , a0{ tunnings.kp + tunnings.ki + tunnings.kd }
        , a1{ -tunnings.kp - (tunnings.kd + tunnings.kd) }
        , a2{ tunnings.kd }
    {
        really_assert(limits.min < limits.max);

        driver.Read([this](auto measuredProcessVariable)
            {
                this->driver.ControlAction(Process(measuredProcessVariable));
            });
    }

    template<class QNumberType>
    void PidIncremental<QNumberType>::SetPoint(QNumberType _setPoint)
    {
        this->setPoint.emplace(_setPoint);
    }

    template<class QNumberType>
    void PidIncremental<QNumberType>::Enable()
    {
        u = QNumberType(0.0f);
        u_1 = QNumberType(0.0f);

        e = QNumberType(0.0f);
        e_1 = QNumberType(0.0f);
        e_2 = QNumberType(0.0f);

        driver.Start(sampleTime);
    }

    template<typename QNumberType>
    void PidIncremental<QNumberType>::Disable()
    {
        driver.Stop();
    }

    template<class QNumberType>
    void PidIncremental<QNumberType>::SetLimits(PidIncremental<QNumberType>::PidBase::Limits limits)
    {
        really_assert(limits.max < limits.min);

        this->limits = limits;
    }

    template<class QNumberType>
    void PidIncremental<QNumberType>::SetTunings(PidIncremental<QNumberType>::PidBase::Tunings tunnings)
    {
        a0 = tunnings.kp + tunnings.ki + tunnings.kd;
        a1 = -tunnings.kp - (tunnings.kd + tunnings.kd);
        a2 = tunnings.kd;
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidIncremental<QNumberType>::Clamp(QNumberType input)
    {
        if (input > limits.max)
            return limits.max;

        if (input < limits.min)
            return limits.min;

        return input;
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidIncremental<QNumberType>::Process(QNumberType measuredProcessVariable)
    {
        if (!setPoint.has_value())
            return measuredProcessVariable;

        u_1 = u;
        e_2 = e_1;
        e_1 = e;
        e = *setPoint - measuredProcessVariable;
        u = Clamp(u_1 + a0 * e + a1 * e_1 + a2 * e_2);

        return u;
    }
}

#pragma once

#include "numerical/controllers/interfaces/PidController.hpp"
#include "numerical/controllers/interfaces/PidDriver.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace controllers
{
    template<typename QNumberType>
    class PidIncrementalBase
    {
    public:
        void SetPoint(QNumberType setPoint);
        void SetLimits(PidLimits<QNumberType> limits);
        void SetTunings(PidTunings<QNumberType> tunnings);
        void Enable();
        void Disable();
        QNumberType Process(QNumberType processVariable);

    protected:
        PidIncrementalBase(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits);

    private:
        QNumberType Clamp(QNumberType input);

    private:
        QNumberType setPointValue{};
        bool hasSetPoint = false;
        PidLimits<QNumberType> limits;
        QNumberType a0 = 0;
        QNumberType a1 = 0;
        QNumberType a2 = 0;

        QNumberType u = QNumberType(0.0f);
        QNumberType u_1 = QNumberType(0.0f);

        QNumberType e = QNumberType(0.0f);
        QNumberType e_1 = QNumberType(0.0f);
        QNumberType e_2 = QNumberType(0.0f);
    };

    template<typename QNumberType>
    class PidIncrementalAsynchronous
        : public AsynchronousPidController<QNumberType>
        , private PidIncrementalBase<QNumberType>
    {
    public:
        PidIncrementalAsynchronous(PidDriver<QNumberType>& driver, std::chrono::system_clock::duration sampleTime, PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits);

        // Implementation of AsynchronousPidController
        void SetPoint(QNumberType setPoint) override;
        void SetLimits(PidLimits<QNumberType> limits) override;
        void SetTunings(PidTunings<QNumberType> tunnings) override;
        void Enable() override;
        void Disable() override;

    private:
        PidDriver<QNumberType>& driver;
        std::chrono::system_clock::duration sampleTime;
    };

    template<typename QNumberType>
    class PidIncrementalSynchronous
        : public SynchronousPidController<QNumberType>
        , private PidIncrementalBase<QNumberType>
    {
    public:
        PidIncrementalSynchronous(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits);

        // Implementation of SynchronousPidController
        void SetPoint(QNumberType setPoint) override;
        void SetLimits(PidLimits<QNumberType> limits) override;
        void SetTunings(PidTunings<QNumberType> tunnings) override;
        void Enable() override;
        void Disable() override;
        QNumberType Process(QNumberType processVariable) override;
    };

    ////    Implementation    ////

    template<typename QNumberType>
    PidIncrementalBase<QNumberType>::PidIncrementalBase(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits)
        : limits{ limits }
        , a0{ tunnings.kp + tunnings.ki + tunnings.kd }
        , a1{ -tunnings.kp - (tunnings.kd + tunnings.kd) }
        , a2{ tunnings.kd }
    {
        really_assert(limits.min < limits.max);
    }

    template<class QNumberType>
    void PidIncrementalBase<QNumberType>::SetPoint(QNumberType _setPoint)
    {
        this->setPointValue = _setPoint;
        this->hasSetPoint = true;
    }

    template<class QNumberType>
    void PidIncrementalBase<QNumberType>::Enable()
    {
        u = QNumberType(0.0f);
        u_1 = QNumberType(0.0f);

        e = QNumberType(0.0f);
        e_1 = QNumberType(0.0f);
        e_2 = QNumberType(0.0f);
    }

    template<class QNumberType>
    void PidIncrementalBase<QNumberType>::Disable()
    {
        hasSetPoint = false;
    }

    template<class QNumberType>
    void PidIncrementalBase<QNumberType>::SetLimits(PidLimits<QNumberType> _limits)
    {
        really_assert(_limits.max > _limits.min);

        this->limits = _limits;
    }

    template<class QNumberType>
    void PidIncrementalBase<QNumberType>::SetTunings(PidTunings<QNumberType> tunings)
    {
        a0 = tunings.kp + tunings.ki + tunings.kd;
        a1 = -tunings.kp - (tunings.kd + tunings.kd);
        a2 = tunings.kd;
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidIncrementalBase<QNumberType>::Clamp(QNumberType input)
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
        PidIncrementalBase<QNumberType>::Process(QNumberType processVariable)
    {
        if (!hasSetPoint) [[unlikely]]
            return processVariable;

        u_1 = u;
        e_2 = e_1;
        e_1 = e;
        e = setPointValue - processVariable;
        u = Clamp(u_1 + a0 * e + a1 * e_1 + a2 * e_2);

        return u;
    }

    template<typename QNumberType>
    PidIncrementalAsynchronous<QNumberType>::PidIncrementalAsynchronous(PidDriver<QNumberType>& driver, std::chrono::system_clock::duration sampleTime, PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits)
        : PidIncrementalBase<QNumberType>{ tunnings, limits }
        , driver{ driver }
        , sampleTime{ sampleTime }
    {
        driver.Read([this](auto processVariable)
            {
                this->driver.ControlAction(this->PidIncrementalBase<QNumberType>::Process(processVariable));
            });
    }

    template<class QNumberType>
    void PidIncrementalAsynchronous<QNumberType>::SetPoint(QNumberType _setPoint)
    {
        PidIncrementalBase<QNumberType>::SetPoint(_setPoint);
    }

    template<class QNumberType>
    void PidIncrementalAsynchronous<QNumberType>::Enable()
    {
        PidIncrementalBase<QNumberType>::Enable();
        driver.Start(sampleTime);
    }

    template<typename QNumberType>
    void PidIncrementalAsynchronous<QNumberType>::Disable()
    {
        driver.Stop();
    }

    template<class QNumberType>
    void PidIncrementalAsynchronous<QNumberType>::SetLimits(PidLimits<QNumberType> limits)
    {
        PidIncrementalBase<QNumberType>::SetLimits(limits);
    }

    template<class QNumberType>
    void PidIncrementalAsynchronous<QNumberType>::SetTunings(PidTunings<QNumberType> tunings)
    {
        PidIncrementalBase<QNumberType>::SetTunings(tunings);
    }

    template<class QNumberType>
    PidIncrementalSynchronous<QNumberType>::PidIncrementalSynchronous(PidTunings<QNumberType> tunnings, PidLimits<QNumberType> limits)
        : PidIncrementalBase<QNumberType>{ tunnings, limits }
    {
    }

    template<class QNumberType>
    void PidIncrementalSynchronous<QNumberType>::SetPoint(QNumberType setPoint)
    {
        PidIncrementalBase<QNumberType>::SetPoint(setPoint);
    }

    template<class QNumberType>
    void PidIncrementalSynchronous<QNumberType>::SetLimits(PidLimits<QNumberType> limits)
    {
        PidIncrementalBase<QNumberType>::SetLimits(limits);
    }

    template<class QNumberType>
    void PidIncrementalSynchronous<QNumberType>::SetTunings(PidTunings<QNumberType> tunnings)
    {
        PidIncrementalBase<QNumberType>::SetTunings(tunnings);
    }

    template<class QNumberType>
    void PidIncrementalSynchronous<QNumberType>::Enable()
    {
        PidIncrementalBase<QNumberType>::Enable();
    }

    template<class QNumberType>
    void PidIncrementalSynchronous<QNumberType>::Disable()
    {
        PidIncrementalBase<QNumberType>::Disable();
    }

    template<class QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        PidIncrementalSynchronous<QNumberType>::Process(QNumberType processVariable)
    {
        return PidIncrementalBase<QNumberType>::Process(processVariable);
    }
}

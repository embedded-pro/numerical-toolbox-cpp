#ifndef CONTROLLERS_PID_HPP
#define CONTROLLERS_PID_HPP

#include "math/QNumber.hpp"
#include <chrono>
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

        Pid(Tunnings tunnings, std::chrono::microseconds sampleTime, Limits limits, bool autoMode = true, bool proportionalOnMeasurement = false, bool differentialOnMeasurement = true);

        void SetPoint(QNumberType setPoint);

        QNumberType Process(QNumberType measuredProcessVariable);

        void Enable();
        void Disable();

        void SetLimits(Limits limits);
        void SetTunnings(Tunnings tunnings);
        void SetSampleTime(std::chrono::microseconds sampleTime);

        void Reset();

    private:
        QNumberType Clamp(QNumberType input);

        void CalculateProportional(QNumberType& error, QNumberType& derivativeInput);
        void CalculateIntegral(QNumberType& error);
        void CalculateDerivative(QNumberType& derivativeInput, QNumberType& derivativeError);

        void UpdateLasts(QNumberType& controllerOutput, QNumberType& error, QNumberType& measuredProcessVariable);

    private:
        template<typename T>
        struct DurationConverter 
        {
            static T FromDuration(std::chrono::microseconds duration) 
            {
                if constexpr (math::is_qnumber<T>::value)
                    return T(static_cast<float>(duration.count()) * 1e-6f);
                else
                    return static_cast<T>(duration.count()) / static_cast<T>(1000000); // from microseconds to seconds
            }
        };

    private:
        Tunnings tunnings;
        std::chrono::microseconds sampleTime;
        Limits limits;
        bool autoMode;
        bool proportionalOnMeasurement;
        bool differentialOnMeasurement;

        std::optional<QNumberType> setPoint;
        QNumberType proportional = 0;
        QNumberType integral = 0;
        QNumberType derivative = 0;

        std::optional<QNumberType> lastControllerOutput;
        std::optional<QNumberType> lastError;
        std::optional<QNumberType> lastMeasuredProcessVariable;
    };

    ////    Implementation    ////

    template<typename QNumberType>
    Pid<QNumberType>::Pid(Tunnings tunnings, std::chrono::microseconds sampleTime, Limits limits, bool autoMode, bool proportionalOnMeasurement, bool differentialOnMeasurement)
        : tunnings(tunnings)
        , sampleTime(sampleTime)
        , limits(limits)
        , autoMode(autoMode)
        , proportionalOnMeasurement(proportionalOnMeasurement)
        , differentialOnMeasurement(differentialOnMeasurement)
    {
        integral = Clamp(0);

        really_assert(limits.min < limits.max);
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
        {
            Reset();

            integral = Clamp(lastControllerOutput.value_or(0));
        }

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
        this->limits = limits;

        if (limits.max >= limits.min)
            std::abort();
    }

    template<class QNumberType>
    void Pid<QNumberType>::SetTunnings(Tunnings tunnings)
    {
        this->tunnings = tunnings;
    }

    template<class QNumberType>
    void Pid<QNumberType>::SetSampleTime(std::chrono::microseconds sampleTime)
    {
        this->sampleTime = sampleTime;

        Reset();

        integral = Clamp(lastControllerOutput.value_or(0));
    }

    template<class QNumberType>
    QNumberType Pid<QNumberType>::Clamp(QNumberType input)
    {
        if (input > limits.max)
            return limits.max;

        if (input < limits.min)
            return limits.min;

        return input;
    }

    template<class QNumberType>
    QNumberType Pid<QNumberType>::Process(QNumberType measuredProcessVariable)
    {
        if (!setPoint || !autoMode)
            return lastControllerOutput.value_or(*setPoint);

        QNumberType error = *setPoint - measuredProcessVariable;
        QNumberType derivativeInput = measuredProcessVariable - lastMeasuredProcessVariable.value_or(measuredProcessVariable);
        QNumberType derivativeError = error - lastError.value_or(error);

        CalculateProportional(error, derivativeInput);
        CalculateIntegral(error);
        CalculateDerivative(derivativeInput, derivativeError);

        QNumberType controllerOutput = Clamp(proportional + integral + derivative);

        UpdateLasts(controllerOutput, error, measuredProcessVariable);

        return controllerOutput;
    }

    template<class QNumberType>
    void Pid<QNumberType>::CalculateProportional(QNumberType& error, QNumberType& derivativeInput)
    {
        if (proportionalOnMeasurement)
            proportional = tunnings.kp * error;
        else
            proportional -= tunnings.kp * derivativeInput;
    }

    template<class QNumberType>
    void Pid<QNumberType>::CalculateIntegral(QNumberType& error)
    {
        integral += tunnings.ki * error * DurationConverter<QNumberType>::FromDuration(sampleTime);
        integral = Clamp(integral);
    }

    template<class QNumberType>
    void Pid<QNumberType>::CalculateDerivative(QNumberType& derivativeInput, QNumberType& derivativeError)
    {
        if (differentialOnMeasurement)
            derivative = -tunnings.kd * derivativeInput / DurationConverter<QNumberType>::FromDuration(sampleTime);
        else
            derivative -= tunnings.kd * derivativeError / DurationConverter<QNumberType>::FromDuration(sampleTime);
    }

    template<class QNumberType>
    void Pid<QNumberType>::UpdateLasts(QNumberType& controllerOutput, QNumberType& error, QNumberType& measuredProcessVariable)
    {
        this->lastControllerOutput.emplace(controllerOutput);
        this->lastError.emplace(error);
        this->lastMeasuredProcessVariable.emplace(measuredProcessVariable);
    }
}

#endif

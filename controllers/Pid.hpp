#ifndef EMBEDDED_PID_HPP
#define EMBEDDED_PID_HPP

#include "infra/util/Optional.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <chrono>
#include <cstdlib>

namespace control_toolbox
{
    template<class T>
    class Pid;

    using PidI32 = Pid<int32_t>;
    using PidFloat = Pid<float>;

    template<class T>
    class Pid
    {
    public:
        struct Tunnings
        {
            T kp;
            T ki;
            T kd;
        };

        struct Limits
        {
            T min;
            T max;
        };

        Pid(Tunnings tunnings, std::chrono::microseconds sampleTime, Limits limits, bool autoMode = true, bool proportionalOnMeasurement = false, bool differentialOnMeasurement = true);

        void SetPoint(T setPoint);

        T Process(T measuredProcessVariable);

        void Enable();
        void Disable();

        void SetLimits(Limits limits);
        void SetTunnings(Tunnings tunnings);
        void SetSampleTime(std::chrono::microseconds sampleTime);

        void Reset();

    private:
        T Clamp(T input);

        void CalculateProportional(T& error, T& derivativeInput);
        void CalculateIntegral(T& error);
        void CalculateDerivative(T& derivativeInput, T& derivativeError);

        void UpdateLasts(T& controllerOutput, T& error, T& measuredProcessVariable);

    private:
        Tunnings tunnings;
        std::chrono::microseconds sampleTime;
        Limits limits; 
        bool autoMode;
        bool proportionalOnMeasurement;
        bool differentialOnMeasurement;

        infra::Optional<T> setPoint = infra::none;
        T proportional = 0;
        T integral = 0;
        T derivative = 0;

        infra::Optional<T> lastControllerOutput = infra::none;
        infra::Optional<T> lastError = infra::none;
        infra::Optional<T> lastMeasuredProcessVariable = infra::none;
    };

    ////    Implementation    ////

    template<class T>
    Pid<T>::Pid(Tunnings tunnings, std::chrono::microseconds sampleTime, Limits limits, bool autoMode, bool proportionalOnMeasurement, bool differentialOnMeasurement)
        : tunnings(tunnings)
        , sampleTime(sampleTime)
        , limits(limits)
        , autoMode(autoMode)
        , proportionalOnMeasurement(proportionalOnMeasurement)
        , differentialOnMeasurement(differentialOnMeasurement)
    {
        integral = Clamp(0);

        really_assert(limits.max >= limits.min);
    }

    template<class T>
    void Pid<T>::SetPoint(T setPoint)
    {
        this->setPoint.Emplace(setPoint);
    }

    template<class T>
    void Pid<T>::Enable()
    {
        if (autoMode)
        {
            Reset();

            integral = Clamp(lastControllerOutput.ValueOr(0));
        }

        autoMode = true;
    }

    template<class T>
    void Pid<T>::Disable()
    {
        autoMode = false;
    }

    template<class T>
    void Pid<T>::SetLimits(Limits limits)
    {
        this->limits = limits;

        really_assert(limits.max >= limits.min);
    }

    template<class T>
    void Pid<T>::SetTunnings(Tunnings tunnings)
    {
        this->tunnings = tunnings;
    }

    template<class T>
    void Pid<T>::SetSampleTime(std::chrono::microseconds sampleTime)
    {
        this->sampleTime = sampleTime;

        Reset();

        integral = Clamp(lastControllerOutput.ValueOr(0));
    }

    template<class T>
    T Pid<T>::Clamp(T input)
    {
        if (input > limits.max)
            return limits.max;

        if (input < limits.min)
            return limits.min;

        return input;
    }

    template<class T>
    T Pid<T>::Process(T measuredProcessVariable)
    {
        if (!setPoint || !autoMode)
            return lastControllerOutput.ValueOr(*setPoint);

        T error = *setPoint - measuredProcessVariable;
        T derivativeInput = measuredProcessVariable - lastMeasuredProcessVariable.ValueOr(measuredProcessVariable);
        T derivativeError = error - lastError.ValueOr(error);

        CalculateProportional(error, derivativeInput);
        CalculateIntegral(error);
        CalculateDerivative(derivativeInput, derivativeError);

        T controllerOutput = Clamp(proportional + integral + derivative);

        UpdateLasts(controllerOutput, error, measuredProcessVariable);

        return controllerOutput;
    }

    template<class T>
    void Pid<T>::CalculateProportional(T& error, T& derivativeInput)
    {
        if (proportionalOnMeasurement)
            proportional = tunnings.kp * error;
        else
            proportional -= tunnings.kp * derivativeInput;
    }

    template<class T>
    void Pid<T>::CalculateIntegral(T& error)
    {
        integral += tunnings.ki * error * static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(sampleTime).count());
        integral = Clamp(integral);
    }

    template<class T>
    void Pid<T>::CalculateDerivative(T& derivativeInput, T& derivativeError)
    {
        if (differentialOnMeasurement)
            derivative = -tunnings.kd * derivativeInput / static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(sampleTime).count());
        else
            derivative -= tunnings.kd * derivativeError / static_cast<T>(std::chrono::duration_cast<std::chrono::microseconds>(sampleTime).count());
    }

    template<class T>
    void Pid<T>::UpdateLasts(T& controllerOutput, T& error, T& measuredProcessVariable)
    {
        this->lastControllerOutput.Emplace(controllerOutput);
        this->lastError.Emplace(error);
        this->lastMeasuredProcessVariable.Emplace(measuredProcessVariable);
    }
}

#endif

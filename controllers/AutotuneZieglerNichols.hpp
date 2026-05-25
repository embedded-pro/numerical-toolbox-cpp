#ifndef CONTROLLERS_AUTOTUNER_ZIEGLER_NICHOLS_HPP
#define CONTROLLERS_AUTOTUNER_ZIEGLER_NICHOLS_HPP

#include "Pid.hpp"
#include "infra/util/Function.hpp"
#include <vector>

namespace controllers
{
    template<typename QNumberType>
    class AutotuneZieglerNichols
    {
    public:
        struct TuningState
        {
            QNumberType currentKp;
            std::vector<QNumberType> oscillationPeaks;
            QNumberType lastValue;
            bool isRising;
            uint32_t stepCount;
            uint32_t oscillationCount;
        };

        struct AutotuneResult
        {
            bool success;
            typename Pid<QNumberType>::Tunnings tunnings;
            QNumberType ultimateGain;   // Ku
            QNumberType ultimatePeriod; // Tu
        };

        AutotuneZieglerNichols(QNumberType initialKp, QNumberType kpStep, QNumberType targetAmplitude, uint32_t minOscillations, uint32_t maxSteps);
        AutotuneResult Tune(Pid<QNumberType>& pid, const infra::Function<QNumberType()>& getMeasurement);

    private:
        void DetectOscillation(TuningState& state, QNumberType currentValue) const;
        QNumberType GetOscillationAmplitude(const TuningState& state);
        AutotuneResult CalculatePIDParameters(const TuningState& state);

    private:
        QNumberType initialKp;
        QNumberType kpStep;
        QNumberType targetAmplitude;
        uint32_t minOscillations;
        uint32_t maxSteps;
    };

    // Implementation //

    template<typename QNumberType>
    AutotuneZieglerNichols<QNumberType>::AutotuneZieglerNichols(QNumberType initialKp, QNumberType kpStep, QNumberType targetAmplitude, uint32_t minOscillations, uint32_t maxSteps)
        : initialKp(initialKp)
        , kpStep(kpStep)
        , targetAmplitude(targetAmplitude)
        , minOscillations(minOscillations)
        , maxSteps(maxSteps)
    {}

    template<typename QNumberType>
    typename AutotuneZieglerNichols<QNumberType>::AutotuneResult AutotuneZieglerNichols<QNumberType>::Tune(Pid<QNumberType>& pid, const infra::Function<QNumberType()>& getMeasurement)
    {
        TuningState state{
            .currentKp = initialKp,
            .lastValue = QNumberType(0),
            .isRising = false,
            .stepCount = 0,
            .oscillationCount = 0
        };

        pid.SetTunnings({ state.currentKp, QNumberType(0), QNumberType(0) });

        while (state.stepCount < maxSteps)
        {
            auto measurement = getMeasurement();
            DetectOscillation(state, measurement);

            if (state.oscillationPeaks.size() >= minOscillations * 2)
                return CalculatePIDParameters(state);

            if (state.stepCount % 100 == 0 && GetOscillationAmplitude(state) < targetAmplitude)
            {
                state.currentKp += kpStep;
                pid.SetTunnings({ state.currentKp, QNumberType(0), QNumberType(0) });
            }

            state.stepCount++;
        }

        return { false, { QNumberType(0), QNumberType(0), QNumberType(0) }, QNumberType(0), QNumberType(0) };
    }

    template<typename QNumberType>
    void AutotuneZieglerNichols<QNumberType>::DetectOscillation(TuningState& state, QNumberType currentValue) const
    {
        bool wasRising = state.isRising;
        state.isRising = currentValue > state.lastValue;

        if (wasRising != state.isRising)
            state.oscillationPeaks.push_back(state.lastValue);

        state.lastValue = currentValue;
    }

    template<typename QNumberType>
    QNumberType AutotuneZieglerNichols<QNumberType>::GetOscillationAmplitude(const TuningState& state)
    {
        if (state.oscillationPeaks.size() < 2)
            return QNumberType(0);

        QNumberType maxPeak = state.oscillationPeaks[0];
        QNumberType minPeak = state.oscillationPeaks[0];

        for (const auto& peak : state.oscillationPeaks)
        {
            if (peak > maxPeak)
                maxPeak = peak;
            if (peak < minPeak)
                minPeak = peak;
        }

        return maxPeak - minPeak;
    }

    template<typename QNumberType>
    typename AutotuneZieglerNichols<QNumberType>::AutotuneResult AutotuneZieglerNichols<QNumberType>::CalculatePIDParameters(const TuningState& state)
    {
        QNumberType Ku = state.currentKp;

        QNumberType Tu = QNumberType(0);
        int periodCount = 0;

        for (size_t i = 0; i < state.oscillationPeaks.size() - 2; i += 2)
        {
            Tu += QNumberType(2);
            periodCount++;
        }

        Tu /= QNumberType(periodCount);

        QNumberType Kp = QNumberType(0.6) * Ku;        // 0.6 Ku
        QNumberType Ki = QNumberType(1.2) * Ku / Tu;   // 2 Kp / Tu
        QNumberType Kd = QNumberType(0.075) * Ku * Tu; // Kp * Tu / 8

        return {
            true,
            { Kp, Ki, Kd },
            Ku,
            Tu
        };
    }
}

#endif

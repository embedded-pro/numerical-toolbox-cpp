#ifndef CONTROLLERS_SPACE_VECTOR_MODULATION_HPP
#define CONTROLLERS_SPACE_VECTOR_MODULATION_HPP

#include "numerical/controllers/TransformsClarkePark.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename QNumberType>
    class SpaceVectorModulation
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point_v<QNumberType>,
            "SpaceVectorModulation can only be instantiated with math::QNumber or floating point types.");

    public:
        struct Output
        {
            QNumberType a;
            QNumberType b;
            QNumberType c;
        };

        OPTIMIZE_FOR_SPEED
        Output Generate(const TwoPhase<QNumberType>& voltagePhase)
        {
            auto pattern = CalculateSwitchingTimes(voltagePhase.alpha * sqrt3Const, voltagePhase.beta * sqrt3Const);

            return Output{
                ClampDutyCycle(pattern.ta),
                ClampDutyCycle(pattern.tb),
                ClampDutyCycle(pattern.tc)
            };
        }

    private:
        OPTIMIZE_FOR_SPEED
        QNumberType ClampDutyCycle(QNumberType duty)
        {
            if (duty < zero)
                return zero;
            if (duty > one)
                return one;

            return duty;
        }

        struct SwitchingPattern
        {
            QNumberType ta;
            QNumberType tb;
            QNumberType tc;
        };

        OPTIMIZE_FOR_SPEED
        SwitchingPattern AddCommonInjection(SwitchingPattern pattern)
        {
            static const QNumberType _half = QNumberType(0.5f);

            QNumberType t0 = one - pattern.ta - pattern.tb - pattern.tc;
            QNumberType t_com = t0 * _half;

            pattern.ta += t_com;
            pattern.tb += t_com;
            pattern.tc += t_com;

            return pattern;
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate60To120Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = valpha + vbeta * invSqrt3;
            QNumberType t2 = -valpha + vbeta * invSqrt3;
            return AddCommonInjection({ t2, t1 + t2, zero });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate120To180Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = -valpha + vbeta * invSqrt3;
            QNumberType t2 = -valpha - vbeta * invSqrt3;
            return AddCommonInjection({ zero, t1 + t2, t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate180To240Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = -valpha - vbeta * invSqrt3;
            QNumberType t2 = vbeta * twoDivSqrt3;
            return AddCommonInjection({ zero, t2, t1 + t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate240To300Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = -valpha + vbeta * invSqrt3;
            QNumberType t2 = -vbeta * twoDivSqrt3;
            return AddCommonInjection({ t2, zero, t1 + t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate300To360Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = valpha + vbeta * invSqrt3;
            QNumberType t2 = -valpha - vbeta * invSqrt3;
            return AddCommonInjection({ t1 + t2, zero, t2 });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern Calculate0To60Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = valpha - vbeta * invSqrt3;
            QNumberType t2 = vbeta * twoDivSqrt3;
            return AddCommonInjection({ t1 + t2, t2, zero });
        }

        OPTIMIZE_FOR_SPEED
        SwitchingPattern CalculateSwitchingTimes(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType v_ref_60 = (valpha * half - vbeta * sqrt3Div2);
            QNumberType v_ref_120 = (-valpha * half - vbeta * sqrt3Div2);

            if (v_ref_60 >= zero)
                return Calculate60To120Degrees(valpha, vbeta);
            if (v_ref_120 >= zero)
                return Calculate120To180Degrees(valpha, vbeta);
            if (-valpha >= zero)
                return Calculate180To240Degrees(valpha, vbeta);
            if (-v_ref_60 >= zero)
                return Calculate240To300Degrees(valpha, vbeta);
            if (-v_ref_120 >= zero)
                return Calculate300To360Degrees(valpha, vbeta);

            return Calculate0To60Degrees(valpha, vbeta);
        }

        QNumberType zero{ std::is_floating_point_v<QNumberType> ? QNumberType(0.0f) : QNumberType(0.0f) };
        QNumberType one{ std::is_floating_point_v<QNumberType> ? QNumberType(1.0f) : QNumberType(0.2f) };
        QNumberType half{ std::is_floating_point_v<QNumberType> ? QNumberType(0.5f) : QNumberType(0.1f) };
        QNumberType sqrt3Const{ std::is_floating_point_v<QNumberType> ? QNumberType(1.732050807568877f) : QNumberType(0.3464101615137754f) };
        QNumberType invSqrt3{ std::is_floating_point_v<QNumberType> ? QNumberType(0.577350269189625f) : QNumberType(0.115470053837925f) };
        QNumberType sqrt3Div2{ std::is_floating_point_v<QNumberType> ? QNumberType(0.866025403784438f) : QNumberType(0.1732050807568876f) };
        QNumberType twoDivSqrt3{ std::is_floating_point_v<QNumberType> ? QNumberType(1.154700538379252f) : QNumberType(0.2309401076758504f) };
    };
}

#endif

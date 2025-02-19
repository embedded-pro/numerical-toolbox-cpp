#ifndef CONTROLLERS_SPACE_VECTOR_MODULATION_HPP
#define CONTROLLERS_SPACE_VECTOR_MODULATION_HPP

#include "toolbox/controllers/TransformsClarkePark.hpp"
#include "toolbox/math/QNumber.hpp"

namespace controllers
{
    template<typename QNumberType>
    class SpaceVectorModulation
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "SpaceVectorModulation can only be instantiated with math::QNumber or floating point types.");

    public:
        struct Output
        {
            QNumberType a;
            QNumberType b;
            QNumberType c;
        };

        explicit SpaceVectorModulation(const math::TrigonometricFunctions<QNumberType>& trigFunctions)
            : trigFunctions(trigFunctions)
        {
            InitializeConstants();
        }

        Output Generate(const RotatingFrame<QNumberType>& dqVoltage, QNumberType scaledTheta)
        {
            really_assert(scaledTheta >= math::Lowest<QNumberType>() && scaledTheta <= math::Max<QNumberType>());

            QNumberType d = dqVoltage.d * outputScale;
            QNumberType q = dqVoltage.q * outputScale;

            QNumberType cosTheta = trigFunctions.Cosine(scaledTheta);
            QNumberType sinTheta = trigFunctions.Sine(scaledTheta);

            QNumberType valpha = d * cosTheta - q * sinTheta;
            QNumberType vbeta = d * sinTheta + q * cosTheta;

            auto pattern = CalculateSwitchingTimes(valpha, vbeta);

            return Output{
                ClampDutyCycle(pattern.ta),
                ClampDutyCycle(pattern.tb),
                ClampDutyCycle(pattern.tc)
            };
        }

    private:
        void InitializeConstants()
        {
            if constexpr (std::is_floating_point<QNumberType>::value)
            {
                zero = QNumberType(0.0f);
                one = QNumberType(1.0f);
                half = QNumberType(0.5f);
                invSqrt3 = QNumberType(0.577350269189625f);
                sqrt3Div2 = QNumberType(0.866025403784438f);
                twoDivSqrt3 = QNumberType(1.154700538379252f);
                outputScale = QNumberType(1.0f);
            }
            else
            {
                zero = QNumberType(0.0f);
                one = QNumberType(0.2f);
                half = QNumberType(0.1f);
                invSqrt3 = QNumberType(0.115470053837925f);
                sqrt3Div2 = QNumberType(0.1732050807568876f);
                twoDivSqrt3 = QNumberType(0.2309401076758504f);
                outputScale = QNumberType(0.2f);
            }
        }

        QNumberType ClampDutyCycle(QNumberType duty)
        {
            if (duty < zero)
                return zero;
            if (duty > one)
                return one / outputScale;

            return duty / outputScale;
        }

        struct SwitchingPattern
        {
            QNumberType ta;
            QNumberType tb;
            QNumberType tc;
        };

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

        SwitchingPattern Calculate60To120Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = valpha + vbeta * invSqrt3;
            QNumberType t2 = -valpha + vbeta * invSqrt3;
            return AddCommonInjection({ t2, t1 + t2, zero });
        }

        SwitchingPattern Calculate120To180Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = -valpha + vbeta * invSqrt3;
            QNumberType t2 = -valpha - vbeta * invSqrt3;
            return AddCommonInjection({ zero, t1 + t2, t2 });
        }

        SwitchingPattern Calculate180To240Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = -valpha - vbeta * invSqrt3;
            QNumberType t2 = vbeta * twoDivSqrt3;
            return AddCommonInjection({ zero, t2, t1 + t2 });
        }

        SwitchingPattern Calculate240To300Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = -valpha + vbeta * invSqrt3;
            QNumberType t2 = -vbeta * twoDivSqrt3;
            return AddCommonInjection({ t2, zero, t1 + t2 });
        }

        SwitchingPattern Calculate300To360Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = valpha + vbeta * invSqrt3;
            QNumberType t2 = -valpha - vbeta * invSqrt3;
            return AddCommonInjection({ t1 + t2, zero, t2 });
        }

        SwitchingPattern Calculate0To60Degrees(QNumberType valpha, QNumberType vbeta)
        {
            QNumberType t1 = valpha - vbeta * invSqrt3;
            QNumberType t2 = vbeta * twoDivSqrt3;
            return AddCommonInjection({ t1 + t2, t2, zero });
        }

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

        const math::TrigonometricFunctions<QNumberType>& trigFunctions;
        QNumberType zero;
        QNumberType one;
        QNumberType half;
        QNumberType invSqrt3;
        QNumberType sqrt3Div2;
        QNumberType twoDivSqrt3;
        QNumberType outputScale;
    };
}

#endif

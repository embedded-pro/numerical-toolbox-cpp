#ifndef CONTROLLERS_SPACE_VECTOR_MODULATION_HPP
#define CONTROLLERS_SPACE_VECTOR_MODULATION_HPP

#include "controllers/TransformsClarkePark.hpp"
#include "math/QNumber.hpp"
#include <algorithm>

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

        explicit SpaceVectorModulation(const dsp::TrigonometricFunctions<QNumberType>& trigFunctions)
            : trigFunctions(trigFunctions)
        {
            InitializeConstants();
        }

        Output Generate(const RotatingFrame<QNumberType>& dqVoltage, QNumberType theta)
        {
            QNumberType cosTheta = trigFunctions.Cosine(theta);
            QNumberType sinTheta = trigFunctions.Sine(theta);

            QNumberType valpha = dqVoltage.d * cosTheta - dqVoltage.q * sinTheta;
            QNumberType vbeta = dqVoltage.d * sinTheta + dqVoltage.q * cosTheta;

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
                oneHalf = QNumberType(0.5f);
                invSqrt3 = QNumberType(0.577350269189625f);
                sqrt3Div2 = QNumberType(0.866025403784438f);
                twoDivSqrt3 = QNumberType(1.154700538379252f);
                outputScale = QNumberType(1.0f);
            }
            else
            {
                // Scale = 0.2f
                zero = QNumberType(0.0f);
                one = QNumberType(0.2f);
                oneHalf = QNumberType(0.1f);
                invSqrt3 = QNumberType(0.115470053837925f);
                sqrt3Div2 = QNumberType(0.1732050807568876f);
                twoDivSqrt3 = QNumberType(0.2309401076758504f);
                outputScale = QNumberType(0.04f);
            }
        }

        QNumberType CreateValue(float value)
        {
            if constexpr (std::is_floating_point<QNumberType>::value)
                return value;
            else
                return QNumberType(std::clamp(value, -0.9f, 0.9f));
        }

        template<typename T>
        float ToFloat(T value)
        {
            if constexpr (std::is_same_v<T, float>)
                return value;
            else
                return value.ToFloat();
        }

        QNumberType ClampDutyCycle(QNumberType duty)
        {
            if constexpr (std::is_floating_point<QNumberType>::value)
                return std::clamp(duty, zero, one);
            else
            {
                if (ToFloat(duty) < ToFloat(zero))
                    return zero;
                if (ToFloat(duty) > ToFloat(one))
                    return one / outputScale;
                return duty / outputScale;
            }
        }

        struct SwitchingPattern
        {
            QNumberType ta;
            QNumberType tb;
            QNumberType tc;
        };

        SwitchingPattern AddCommonInjection(SwitchingPattern pattern)
        {
            QNumberType t0 = one - pattern.ta - pattern.tb - pattern.tc;
            QNumberType t_com = t0 * oneHalf;

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
            QNumberType v_ref_60 = (valpha * oneHalf - vbeta * sqrt3Div2);
            QNumberType v_ref_120 = (-valpha * oneHalf - vbeta * sqrt3Div2);

            QNumberType valphaAbs = ToFloat(valpha) >= 0 ? valpha : QNumberType(-ToFloat(valpha));
            QNumberType compareThreshold = valphaAbs * QNumberType(0.1f);

            if (ToFloat(v_ref_60) >= ToFloat(compareThreshold))
                return Calculate60To120Degrees(valpha, vbeta);
            if (ToFloat(v_ref_120) >= ToFloat(compareThreshold))
                return Calculate120To180Degrees(valpha, vbeta);
            if (ToFloat(-valpha) >= ToFloat(compareThreshold))
                return Calculate180To240Degrees(valpha, vbeta);
            if (ToFloat(-v_ref_60) >= ToFloat(compareThreshold))
                return Calculate240To300Degrees(valpha, vbeta);
            if (ToFloat(-v_ref_120) >= ToFloat(compareThreshold))
                return Calculate300To360Degrees(valpha, vbeta);

            return Calculate0To60Degrees(valpha, vbeta);
        }

        const dsp::TrigonometricFunctions<QNumberType>& trigFunctions;
        QNumberType zero;
        QNumberType one;
        QNumberType oneHalf;
        QNumberType invSqrt3;
        QNumberType sqrt3Div2;
        QNumberType twoDivSqrt3;
        QNumberType outputScale;
    };
}

#endif

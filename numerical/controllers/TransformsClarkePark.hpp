#ifndef TRANSFORMS_CLARKE_PARK_HPP
#define TRANSFORMS_CLARKE_PARK_HPP

#include "numerical/math/AdvancedFunctions.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/TrigonometricFunctions.hpp"

namespace controllers
{
    template<typename QNumberType>
    struct ThreePhase
    {
        QNumberType a;
        QNumberType b;
        QNumberType c;
    };

    template<typename QNumberType>
    struct TwoPhase
    {
        QNumberType alpha;
        QNumberType beta;
    };

    template<typename QNumberType>
    struct RotatingFrame
    {
        QNumberType d;
        QNumberType q;
    };

    template<typename QNumberType>
    class Clarke
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Clarke can only be instantiated with math::QNumber or floating point types.");

    public:
        ALWAYS_INLINE_HOT
        TwoPhase<QNumberType> Forward(const ThreePhase<QNumberType>& input)
        {
            const QNumberType bc_sum = input.b + input.c;

            return TwoPhase<QNumberType>{ twoThirds * (input.a - oneHalf * bc_sum), invSqrt3 * (input.b - input.c) };
        }

        ALWAYS_INLINE_HOT
        ThreePhase<QNumberType> Inverse(const TwoPhase<QNumberType>& input)
        {
            const QNumberType alpha_half = oneHalf * input.alpha;
            const QNumberType beta_sqrt3_half = sqrt3Div2 * input.beta;
            return ThreePhase<QNumberType>{ input.alpha, -alpha_half + beta_sqrt3_half, -alpha_half - beta_sqrt3_half };
        }

    private:
        QNumberType oneHalf = QNumberType(0.5f);
        QNumberType twoThirds = QNumberType(0.666666667f);
        QNumberType invSqrt3 = QNumberType(0.577350269f);
        QNumberType sqrt3Div2 = QNumberType(0.8660254037f);
    };

    template<typename QNumberType>
    class Park
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Park can only be instantiated with math::QNumber or floating point types.");

    public:
        explicit Park(const math::TrigonometricFunctions<QNumberType>& trigFunctions)
            : trigFunctions(trigFunctions)
        {}

        ALWAYS_INLINE_HOT
        RotatingFrame<QNumberType> Forward(const TwoPhase<QNumberType>& input, QNumberType scaledTheta)
        {
            really_assert(scaledTheta >= math::Lowest<QNumberType>() && scaledTheta <= math::Max<QNumberType>());

            const QNumberType cosTheta = trigFunctions.Cosine(scaledTheta);
            const QNumberType sinTheta = trigFunctions.Sine(scaledTheta);

            const QNumberType alpha_cos = input.alpha * cosTheta;
            const QNumberType beta_sin = input.beta * sinTheta;
            const QNumberType alpha_sin = input.alpha * sinTheta;
            const QNumberType beta_cos = input.beta * cosTheta;

            return RotatingFrame<QNumberType>{ alpha_cos + beta_sin, -alpha_sin + beta_cos };
        }

        ALWAYS_INLINE_HOT
        TwoPhase<QNumberType> Inverse(const RotatingFrame<QNumberType>& input, QNumberType scaledTheta)
        {
            really_assert(scaledTheta >= math::Lowest<QNumberType>() && scaledTheta <= math::Max<QNumberType>());

            const QNumberType cosTheta = trigFunctions.Cosine(scaledTheta);
            const QNumberType sinTheta = trigFunctions.Sine(scaledTheta);

            const QNumberType d_cos = input.d * cosTheta;
            const QNumberType q_sin = input.q * sinTheta;
            const QNumberType d_sin = input.d * sinTheta;
            const QNumberType q_cos = input.q * cosTheta;

            return TwoPhase<QNumberType>{ d_cos - q_sin, d_sin + q_cos };
        }

    private:
        const math::TrigonometricFunctions<QNumberType>& trigFunctions;
    };

    template<typename QNumberType>
    class ClarkePark
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "ClarkePark can only be instantiated with math::QNumber or floating point types.");

    public:
        ClarkePark(const math::TrigonometricFunctions<QNumberType>& trigFunctions)
            : park(trigFunctions)
        {}

        RotatingFrame<QNumberType> Forward(const ThreePhase<QNumberType>& input, QNumberType theta)
        {
            return park.Forward(clarke.Forward(input), theta);
        }

        ThreePhase<QNumberType> Inverse(const RotatingFrame<QNumberType>& input, QNumberType theta)
        {
            return clarke.Inverse(park.Inverse(input, theta));
        }

    private:
        Clarke<QNumberType> clarke;
        Park<QNumberType> park;
    };
}

#endif

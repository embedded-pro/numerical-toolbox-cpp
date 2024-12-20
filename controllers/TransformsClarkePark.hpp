#ifndef TRANSFORMS_CLARKE_PARK_HPP
#define TRANSFORMS_CLARKE_PARK_HPP

#include "math/AdvancedFunctions.hpp"
#include "math/QNumber.hpp"
#include "math/TrigonometricFunctions.hpp"

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
        explicit Clarke(const dsp::AdvancedFunctions<QNumberType>& advancedFunctions)
            : advancedFunctions(advancedFunctions)
        {
            InitializeConstants();
        }

        // Forward Clarke transform (abc -> αβ)
        TwoPhase<QNumberType> Forward(const ThreePhase<QNumberType>& input)
        {
            // Pre-compute b+c term
            const QNumberType bc_sum = input.b + input.c;

            return TwoPhase<QNumberType>{
                // α = 2/3 * (a - 0.5*(b + c))
                twoThirds * (input.a - oneHalf * bc_sum),
                // β = (1/√3) * (b - c)
                sqrt3Div2 * (input.b - input.c)
            };
        }

        // Inverse Clarke transform (αβ -> abc)
        ThreePhase<QNumberType> Inverse(const TwoPhase<QNumberType>& input)
        {
            // Pre-compute shared terms
            const QNumberType alpha_half = oneHalf * input.alpha;
            const QNumberType beta_sqrt3_half = sqrt3Div2 * input.beta;

            return ThreePhase<QNumberType>{
                input.alpha,
                -alpha_half + beta_sqrt3_half,
                -alpha_half - beta_sqrt3_half
            };
        }

    private:
        void InitializeConstants()
        {
            if constexpr (std::is_floating_point<QNumberType>::value)
            {
                oneHalf = QNumberType(0.5);
                twoThirds = QNumberType(0.6667);
                sqrt3Div2 = QNumberType(0.866);
            }
            else
            {
                oneHalf = QNumberType(0.5f);
                twoThirds = QNumberType(0.6667f);
                sqrt3Div2 = QNumberType(0.866f);
            }
        }

        const dsp::AdvancedFunctions<QNumberType>& advancedFunctions;
        QNumberType oneHalf;
        QNumberType twoThirds;
        QNumberType sqrt3Div2;
    };

    template<typename QNumberType>
    class Park
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Park can only be instantiated with math::QNumber or floating point types.");

    public:
        explicit Park(const dsp::TrigonometricFunctions<QNumberType>& trigFunctions)
            : trigFunctions(trigFunctions)
        {}

        // Forward Park transform (αβ -> dq)
        RotatingFrame<QNumberType> Forward(const TwoPhase<QNumberType>& input, QNumberType theta)
        {
            const QNumberType cosTheta = trigFunctions.Cosine(theta);
            const QNumberType sinTheta = trigFunctions.Sine(theta);

            // Pre-compute shared terms
            const QNumberType alpha_cos = input.alpha * cosTheta;
            const QNumberType beta_sin = input.beta * sinTheta;
            const QNumberType alpha_sin = input.alpha * sinTheta;
            const QNumberType beta_cos = input.beta * cosTheta;

            return RotatingFrame<QNumberType>{
                alpha_cos + beta_sin,
                -alpha_sin + beta_cos
            };
        }

        // Inverse Park transform (dq -> αβ)
        TwoPhase<QNumberType> Inverse(const RotatingFrame<QNumberType>& input, QNumberType theta)
        {
            const QNumberType cosTheta = trigFunctions.Cosine(theta);
            const QNumberType sinTheta = trigFunctions.Sine(theta);

            // Pre-compute shared terms
            const QNumberType d_cos = input.d * cosTheta;
            const QNumberType q_sin = input.q * sinTheta;
            const QNumberType d_sin = input.d * sinTheta;
            const QNumberType q_cos = input.q * cosTheta;

            return TwoPhase<QNumberType>{
                d_cos - q_sin,
                d_sin + q_cos
            };
        }

    private:
        const dsp::TrigonometricFunctions<QNumberType>& trigFunctions;
    };

    template<typename QNumberType>
    class ClarkePark
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "ClarkePark can only be instantiated with math::QNumber or floating point types.");

    public:
        ClarkePark(const dsp::TrigonometricFunctions<QNumberType>& trigFunctions,
            const dsp::AdvancedFunctions<QNumberType>& advancedFunctions)
            : clarke(advancedFunctions)
            , park(trigFunctions)
        {}

        // Forward transform (abc -> dq)
        RotatingFrame<QNumberType> Forward(const ThreePhase<QNumberType>& input, QNumberType theta)
        {
            return park.Forward(clarke.Forward(input), theta);
        }

        // Inverse transform (dq -> abc)
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

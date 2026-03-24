#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/QNumber.hpp"

namespace neural_network
{
    template<typename QNumberType>
    class ActivationFunction
    {
        static_assert(math::is_qnumber_v<QNumberType> || std::is_floating_point_v<QNumberType>,
            "ActivationFunction can only be instantiated with math::QNumber types.");

    public:
        virtual QNumberType Forward(QNumberType x) const = 0;
        virtual QNumberType Backward(QNumberType x) const = 0;

        virtual void ForwardVector(QNumberType* output, const QNumberType* input, std::size_t size) const;
        virtual void BackwardVector(QNumberType* result, const QNumberType* preActivation, const QNumberType* output, const QNumberType* outputGradient, std::size_t size) const;
    };

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED void ActivationFunction<QNumberType>::ForwardVector(QNumberType* output, const QNumberType* input, std::size_t size) const
    {
        for (std::size_t i = 0; i < size; ++i)
            output[i] = Forward(input[i]);
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED void ActivationFunction<QNumberType>::BackwardVector(QNumberType* result, const QNumberType* preActivation, const QNumberType* /*output*/, const QNumberType* outputGradient, std::size_t size) const
    {
        for (std::size_t i = 0; i < size; ++i)
            result[i] = outputGradient[i] * Backward(preActivation[i]);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ActivationFunction<float>;
    extern template class ActivationFunction<math::Q15>;
    extern template class ActivationFunction<math::Q31>;
#endif
}

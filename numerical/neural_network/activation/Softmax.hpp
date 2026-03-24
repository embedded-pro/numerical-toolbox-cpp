#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "numerical/neural_network/activation/ActivationFunction.hpp"
#include <cmath>

namespace neural_network
{
    template<typename QNumberType>
    class Softmax
        : public ActivationFunction<QNumberType>
    {
    public:
        QNumberType Forward(QNumberType x) const override;
        QNumberType Backward(QNumberType x) const override;
        void ForwardVector(QNumberType* output, const QNumberType* input, std::size_t size) const override;
        void BackwardVector(QNumberType* result, const QNumberType* preActivation, const QNumberType* output, const QNumberType* outputGradient, std::size_t size) const override;
    };

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
    QNumberType Softmax<QNumberType>::Forward(QNumberType x) const
    {
        return QNumberType(std::exp(math::ToFloat(x)));
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
    QNumberType Softmax<QNumberType>::Backward(QNumberType x) const
    {
        QNumberType y = Forward(x);
        return y * (QNumberType(0.9999f) - y);
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
    void Softmax<QNumberType>::ForwardVector(QNumberType* output, const QNumberType* input, std::size_t size) const
    {
        QNumberType maxVal = input[0];
        for (std::size_t i = 1; i < size; ++i)
            if (input[i] > maxVal)
                maxVal = input[i];

        QNumberType sum = QNumberType(0.0f);
        for (std::size_t i = 0; i < size; ++i)
        {
            output[i] = QNumberType(std::exp(math::ToFloat(input[i]) - math::ToFloat(maxVal)));
            sum += output[i];
        }

        for (std::size_t i = 0; i < size; ++i)
            output[i] = std::max(std::min(output[i] / sum, QNumberType(0.9999f)), QNumberType(0.0001f));
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
    void Softmax<QNumberType>::BackwardVector(QNumberType* result, const QNumberType* /*preActivation*/, const QNumberType* output, const QNumberType* outputGradient, std::size_t size) const
    {
        QNumberType dot = QNumberType(0.0f);
        for (std::size_t i = 0; i < size; ++i)
            dot += outputGradient[i] * output[i];

        for (std::size_t i = 0; i < size; ++i)
            result[i] = output[i] * (outputGradient[i] - dot);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Softmax<float>;
    extern template class Softmax<math::Q15>;
    extern template class Softmax<math::Q31>;
#endif
}

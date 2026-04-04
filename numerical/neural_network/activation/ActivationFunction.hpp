#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/QNumber.hpp"
#include <span>

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

        virtual void ForwardVector(std::span<QNumberType> output, std::span<const QNumberType> input) const;
        virtual void BackwardVector(std::span<QNumberType> result, std::span<const QNumberType> preActivation, std::span<const QNumberType> output, std::span<const QNumberType> outputGradient) const;
    };

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED void ActivationFunction<QNumberType>::ForwardVector(std::span<QNumberType> output, std::span<const QNumberType> input) const
    {
        for (std::size_t i = 0; i < output.size(); ++i)
            output[i] = Forward(input[i]);
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED void ActivationFunction<QNumberType>::BackwardVector(std::span<QNumberType> result, std::span<const QNumberType> preActivation, std::span<const QNumberType> /*output*/, std::span<const QNumberType> outputGradient) const
    {
        for (std::size_t i = 0; i < result.size(); ++i)
            result[i] = outputGradient[i] * Backward(preActivation[i]);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ActivationFunction<float>;
    extern template class ActivationFunction<math::Q15>;
    extern template class ActivationFunction<math::Q31>;
#endif
}

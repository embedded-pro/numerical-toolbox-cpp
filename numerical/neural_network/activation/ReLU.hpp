#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "numerical/neural_network/activation/ActivationFunction.hpp"

namespace neural_network
{
    template<typename QNumberType>
    class ReLU
        : public ActivationFunction<QNumberType>
    {
    public:
        QNumberType Forward(QNumberType x) const override;
        QNumberType Backward(QNumberType x) const override;
    };

    // Implementation

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
    QNumberType ReLU<QNumberType>::Forward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? x : QNumberType(0.0f);
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
    QNumberType ReLU<QNumberType>::Backward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? QNumberType(0.9999f) : QNumberType(0.0f);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ReLU<float>;
    extern template class ReLU<math::Q15>;
    extern template class ReLU<math::Q31>;
#endif
}

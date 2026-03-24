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
    class Sigmoid
        : public ActivationFunction<QNumberType>
    {
    public:
        QNumberType Forward(QNumberType x) const override;
        QNumberType Backward(QNumberType x) const override;
    };

    // Implementation

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        Sigmoid<QNumberType>::Forward(QNumberType x) const
    {
        return QNumberType(1.0f / (1.0f + std::exp(-math::ToFloat(x))));
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        Sigmoid<QNumberType>::Backward(QNumberType x) const
    {
        QNumberType y = Forward(x);
        return y * (QNumberType(0.9999f) - y);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Sigmoid<float>;
    extern template class Sigmoid<math::Q15>;
    extern template class Sigmoid<math::Q31>;
#endif
}

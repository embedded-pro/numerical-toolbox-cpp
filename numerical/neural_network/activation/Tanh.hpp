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
    class Tanh
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
        Tanh<QNumberType>::Forward(QNumberType x) const
    {
        return QNumberType(std::tanh(math::ToFloat(x)));
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        Tanh<QNumberType>::Backward(QNumberType x) const
    {
        QNumberType y = Forward(x);
        return QNumberType(0.9999f) - y * y;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Tanh<float>;
    extern template class Tanh<math::Q15>;
    extern template class Tanh<math::Q31>;
#endif
}

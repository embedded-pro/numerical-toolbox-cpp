#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/neural_network/activation/ActivationFunction.hpp"

namespace neural_network
{
    template<typename QNumberType>
    class LeakyReLU
        : public ActivationFunction<QNumberType>
    {
    public:
        explicit LeakyReLU(QNumberType alpha = QNumberType(0.01f));

        QNumberType Forward(QNumberType x) const override;
        QNumberType Backward(QNumberType x) const override;

    private:
        QNumberType alpha;
    };

    // Implementation

    template<typename QNumberType>
    LeakyReLU<QNumberType>::LeakyReLU(QNumberType alpha)
        : alpha(alpha)
    {}

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        LeakyReLU<QNumberType>::Forward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? x : alpha * x;
    }

    template<typename QNumberType>
    OPTIMIZE_FOR_SPEED
        QNumberType
        LeakyReLU<QNumberType>::Backward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? QNumberType(0.9999f) : alpha;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LeakyReLU<float>;
    extern template class LeakyReLU<math::Q15>;
    extern template class LeakyReLU<math::Q31>;
#endif
}

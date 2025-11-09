#ifndef NEURAL_NETWORK_ACTIVATION_SOFTMAX_HPP
#define NEURAL_NETWORK_ACTIVATION_SOFTMAX_HPP

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
    };

    // Implementation

    template<typename QNumberType>
    QNumberType Softmax<QNumberType>::Forward(QNumberType x) const
    {
        return QNumberType(std::exp(math::ToFloat(x)));
    }

    template<typename QNumberType>
    QNumberType Softmax<QNumberType>::Backward(QNumberType x) const
    {
        QNumberType y = Forward(x);
        return y * (QNumberType(0.9999f) - y);
    }
}

#endif

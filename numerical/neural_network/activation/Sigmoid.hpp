#ifndef NEURAL_NETWORK_ACTIVATION_SIGMOID_HPP
#define NEURAL_NETWORK_ACTIVATION_SIGMOID_HPP

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
    QNumberType Sigmoid<QNumberType>::Forward(QNumberType x) const
    {
        return QNumberType(1.0f / (1.0f + std::exp(-math::ToFloat(x))));
    }

    template<typename QNumberType>
    QNumberType Sigmoid<QNumberType>::Backward(QNumberType x) const
    {
        QNumberType y = Forward(x);
        return y * (QNumberType(0.9999f) - y);
    }
}

#endif

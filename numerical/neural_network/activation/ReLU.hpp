#ifndef NEURAL_NETWORK_ACTIVATION_RELU_HPP
#define NEURAL_NETWORK_ACTIVATION_RELU_HPP

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
    QNumberType ReLU<QNumberType>::Forward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? x : QNumberType(0.0f);
    }

    template<typename QNumberType>
    QNumberType ReLU<QNumberType>::Backward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? QNumberType(0.9999f) : QNumberType(0.0f);
    }
}

#endif

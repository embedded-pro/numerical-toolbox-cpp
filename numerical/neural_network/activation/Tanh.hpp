#ifndef NEURAL_NETWORK_ACTIVATION_TANH_HPP

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
    QNumberType Tanh<QNumberType>::Forward(QNumberType x) const
    {
        return QNumberType(std::tanh(math::ToFloat(x)));
    }

    template<typename QNumberType>
    QNumberType Tanh<QNumberType>::Backward(QNumberType x) const
    {
        QNumberType y = Forward(x);
        return QNumberType(0.9999f) - y * y;
    }
}

#endif

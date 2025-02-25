#ifndef NEURAL_NETWORK_ACTIVATION_LEAKYRELU_HPP
#define NEURAL_NETWORK_ACTIVATION_LEAKYRELU_HPP

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
    QNumberType LeakyReLU<QNumberType>::Forward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? x : alpha * x;
    }

    template<typename QNumberType>
    QNumberType LeakyReLU<QNumberType>::Backward(QNumberType x) const
    {
        return x > QNumberType(0.0f) ? QNumberType(0.9999f) : alpha;
    }
}

#endif

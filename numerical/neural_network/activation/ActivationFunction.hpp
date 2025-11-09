#ifndef NEURAL_NETWORK_ACTIVATION_FUNCTION_HPP
#define NEURAL_NETWORK_ACTIVATION_FUNCTION_HPP

#include "numerical/math/QNumber.hpp"

namespace neural_network
{
    template<typename QNumberType>
    class ActivationFunction
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "ActivationFunction can only be instantiated with math::QNumber types.");

    public:
        virtual QNumberType Forward(QNumberType x) const = 0;
        virtual QNumberType Backward(QNumberType x) const = 0;
    };
}

#endif

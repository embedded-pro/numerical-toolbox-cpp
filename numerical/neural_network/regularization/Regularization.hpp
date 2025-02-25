#ifndef NEURAL_NETWORK_REGULARIZATION_HPP
#define NEURAL_NETWORK_REGULARIZATION_HPP

#include "numerical/math/Matrix.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t Size>
    class Regularization
    {
    public:
        using Vector = math::Vector<QNumberType, Size>;
        virtual QNumberType Calculate(const Vector& parameters) const = 0;
    };
}

#endif

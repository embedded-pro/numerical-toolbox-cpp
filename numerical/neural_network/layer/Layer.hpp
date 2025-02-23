#ifndef NEURAL_NETWORK_LAYER_HPP
#define NEURAL_NETWORK_LAYER_HPP

#include "numerical/math/Matrix.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, std::size_t ParameterSize>
    class Layer
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "Layer can only be instantiated with math::QNumber types.");

    public:
        using InputVector = math::Vector<QNumberType, InputSize>;
        using OutputVector = math::Vector<QNumberType, OutputSize>;
        using ParameterVector = math::Vector<QNumberType, ParameterSize>;

        virtual void Forward(const InputVector& input) = 0;
        virtual InputVector& Backward(const OutputVector& output_gradient) = 0;
        virtual ParameterVector& Parameters() const = 0;
        virtual void SetParameters(const ParameterVector& parameters) = 0;
    };
}

#endif

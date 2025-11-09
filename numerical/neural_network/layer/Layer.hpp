#ifndef NEURAL_NETWORK_LAYER_HPP
#define NEURAL_NETWORK_LAYER_HPP

#include "numerical/math/Matrix.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t InputSize_, std::size_t OutputSize_, std::size_t ParameterSize_>
    class Layer
    {
    public:
        using InputVector = math::Matrix<QNumberType, InputSize_, 1>;
        using OutputVector = math::Matrix<QNumberType, OutputSize_, 1>;
        using ParameterVector = math::Matrix<QNumberType, ParameterSize_, 1>;

        static constexpr std::size_t InputSize = InputSize_;
        static constexpr std::size_t OutputSize = OutputSize_;
        static constexpr std::size_t ParameterSize = ParameterSize_;

        virtual void Forward(const InputVector& input) = 0;
        virtual InputVector& Backward(const OutputVector& output_gradient) = 0;
        virtual ParameterVector& Parameters() const = 0;
        virtual void SetParameters(const ParameterVector& parameters) = 0;
    };
}

#endif

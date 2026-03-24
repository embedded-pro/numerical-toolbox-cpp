#include "numerical/neural_network/activation/ActivationFunction.hpp"

namespace neural_network
{
    template class ActivationFunction<float>;
    template class ActivationFunction<math::Q15>;
    template class ActivationFunction<math::Q31>;
}

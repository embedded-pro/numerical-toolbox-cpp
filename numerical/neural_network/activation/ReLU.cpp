#include "numerical/neural_network/activation/ReLU.hpp"

namespace neural_network
{
    template class ReLU<float>;
    template class ReLU<math::Q15>;
    template class ReLU<math::Q31>;
}

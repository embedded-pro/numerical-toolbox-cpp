#include "numerical/neural_network/activation/Softmax.hpp"

namespace neural_network
{
    template class Softmax<float>;
    template class Softmax<math::Q15>;
    template class Softmax<math::Q31>;
}

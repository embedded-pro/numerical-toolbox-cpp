#include "numerical/neural_network/activation/Sigmoid.hpp"

namespace neural_network
{
    template class Sigmoid<float>;
    template class Sigmoid<math::Q15>;
    template class Sigmoid<math::Q31>;
}

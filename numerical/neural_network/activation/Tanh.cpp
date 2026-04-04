#include "numerical/neural_network/activation/Tanh.hpp"

namespace neural_network
{
    template class Tanh<float>;
    template class Tanh<math::Q15>;
    template class Tanh<math::Q31>;
}

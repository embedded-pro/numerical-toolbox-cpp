#include "numerical/neural_network/activation/LeakyReLU.hpp"

namespace neural_network
{
    template class LeakyReLU<float>;
    template class LeakyReLU<math::Q15>;
    template class LeakyReLU<math::Q31>;
}

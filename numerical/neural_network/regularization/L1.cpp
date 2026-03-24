#include "numerical/neural_network/regularization/L1.hpp"

namespace neural_network
{
    template class L1<float, 4>;
    template class L1<math::Q15, 4>;
    template class L1<math::Q31, 4>;
}

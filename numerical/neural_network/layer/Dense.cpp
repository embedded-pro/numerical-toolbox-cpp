#include "numerical/neural_network/layer/Dense.hpp"

namespace neural_network
{
    template class Dense<float, 3, 2>;
    template class Dense<math::Q15, 3, 2>;
    template class Dense<math::Q31, 3, 2>;
}

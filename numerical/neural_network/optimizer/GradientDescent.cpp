#include "numerical/neural_network/optimizer/GradientDescent.hpp"

namespace neural_network
{
    template class GradientDescent<float, 2>;
    template class GradientDescent<math::Q15, 2>;
    template class GradientDescent<math::Q31, 2>;
}

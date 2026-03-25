#include "numerical/optimization/GradientDescent.hpp"

namespace optimization
{
    template class GradientDescent<float, 2>;
    template class GradientDescent<math::Q15, 2>;
    template class GradientDescent<math::Q31, 2>;
}

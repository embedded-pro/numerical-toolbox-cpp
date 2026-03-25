#include "numerical/regularization/L2.hpp"

namespace regularization
{
    template class L2<float, 4>;
    template class L2<math::Q15, 4>;
    template class L2<math::Q31, 4>;
}

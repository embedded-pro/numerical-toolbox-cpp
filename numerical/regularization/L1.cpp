#include "numerical/regularization/L1.hpp"

namespace regularization
{
    template class L1<float, 4>;
    template class L1<math::Q15, 4>;
    template class L1<math::Q31, 4>;
}

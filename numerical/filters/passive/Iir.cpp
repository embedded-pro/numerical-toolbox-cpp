#include "numerical/filters/passive/Iir.hpp"

namespace filters::passive
{
    template class Iir<float, 3, 3>;
    template class Iir<math::Q15, 3, 3>;
    template class Iir<math::Q31, 3, 3>;
}

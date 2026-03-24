#include "numerical/filters/passive/Fir.hpp"

namespace filters::passive
{
    template class Fir<float, 3>;
    template class Fir<math::Q15, 3>;
    template class Fir<math::Q31, 3>;
}

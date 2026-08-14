#include "numerical/analysis/DiscreteCosineTransform.hpp"
#include "numerical/math/QNumber.hpp"

namespace analysis
{
    template class DiscreteCosineTransform<float, 8>;
    template class DiscreteCosineTransform<math::Q15, 8>;
    template class DiscreteCosineTransform<math::Q31, 8>;
}

#include "numerical/analysis/DiscreteCosineTransform.hpp"
#include "numerical/math/QNumber.hpp"

namespace analysis
{
    template class DiscreteConsineTransform<float, 8>;
    template class DiscreteConsineTransform<math::Q15, 8>;
    template class DiscreteConsineTransform<math::Q31, 8>;
}

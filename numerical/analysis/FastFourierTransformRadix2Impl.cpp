#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/math/QNumber.hpp"

namespace analysis
{
    template class FastFourierTransformRadix2Impl<float, 8>;
    template class FastFourierTransformRadix2Impl<math::Q15, 8>;
    template class FastFourierTransformRadix2Impl<math::Q31, 8>;
}

#include "numerical/analysis/windowing/Windowing.hpp"
#include "numerical/math/QNumber.hpp"

namespace windowing
{
    template class HammingWindow<float>;
    template class HammingWindow<math::Q15>;
    template class HammingWindow<math::Q31>;

    template class HanningWindow<float>;
    template class HanningWindow<math::Q15>;
    template class HanningWindow<math::Q31>;

    template class BlackmanWindow<float>;
    template class BlackmanWindow<math::Q15>;
    template class BlackmanWindow<math::Q31>;

    template class RectangularWindow<float>;
    template class RectangularWindow<math::Q15>;
    template class RectangularWindow<math::Q31>;
}

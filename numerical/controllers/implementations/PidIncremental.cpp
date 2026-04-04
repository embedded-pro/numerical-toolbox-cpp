#include "numerical/controllers/implementations/PidIncremental.hpp"
#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template class PidIncrementalBase<float>;
    template class PidIncrementalBase<math::Q15>;
    template class PidIncrementalBase<math::Q31>;

    template class PidIncrementalAsynchronous<float>;
    template class PidIncrementalAsynchronous<math::Q15>;
    template class PidIncrementalAsynchronous<math::Q31>;

    template class PidIncrementalSynchronous<float>;
    template class PidIncrementalSynchronous<math::Q15>;
    template class PidIncrementalSynchronous<math::Q31>;
}

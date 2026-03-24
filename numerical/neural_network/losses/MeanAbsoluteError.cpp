#include "numerical/neural_network/losses/MeanAbsoluteError.hpp"

namespace neural_network
{
    template class MeanAbsoluteError<float, 2>;
    template class MeanAbsoluteError<math::Q15, 2>;
    template class MeanAbsoluteError<math::Q31, 2>;
}

#include "numerical/neural_network/losses/MeanSquaredError.hpp"

namespace neural_network
{
    template class MeanSquaredError<float, 2>;
    template class MeanSquaredError<math::Q15, 2>;
    template class MeanSquaredError<math::Q31, 2>;
}

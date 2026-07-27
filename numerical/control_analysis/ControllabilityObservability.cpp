#include "numerical/control_analysis/ControllabilityObservability.hpp"

namespace control_analysis
{
    template class ControllabilityObservability<float, 2, 1, 1>;

    template std::size_t ControllabilityObservability<float, 2, 1, 1>::Rank<2, 2>(
        const math::Matrix<float, 2, 2>&, float);
    template float ControllabilityObservability<float, 2, 1, 1>::MaxAbsValue<2, 2>(
        const math::Matrix<float, 2, 2>&);
    template void ControllabilityObservability<float, 2, 1, 1>::SwapRows<2, 2>(
        math::Matrix<float, 2, 2>&, std::size_t, std::size_t);
    template void ControllabilityObservability<float, 2, 1, 1>::EliminateBelow<2, 2>(
        math::Matrix<float, 2, 2>&, std::size_t, std::size_t, float);
}

#include "numerical/analysis/DiscreteWaveletTransform.hpp"

namespace analysis
{
    template class WaveletFilters<float, 2>;
    template class WaveletFilters<float, 4>;
    template class DiscreteWaveletTransform<float, 16, 3, 2>;
    template class DiscreteWaveletTransform<float, 16, 3, 4>;
}

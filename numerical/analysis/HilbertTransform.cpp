#include "numerical/analysis/HilbertTransform.hpp"

namespace analysis
{
    template class AnalyticSignalFft<float, 64>;
    template class HilbertFir<float, 31>;
}

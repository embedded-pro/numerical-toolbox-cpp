#include "numerical/analysis/SignalDetectors.hpp"

namespace analysis
{
    template class PeakHold<float>;
    template class ZeroCrossingCounter<float>;
    template class RmsEnvelope<float>;
}

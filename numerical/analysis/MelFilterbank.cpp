#include "numerical/analysis/MelFilterbank.hpp"

namespace analysis
{
    template float HzToMel<float>(float, MelScale);
    template float MelToHz<float>(float, MelScale);
    template class MelFilterbank<float, 256, 20>;
}

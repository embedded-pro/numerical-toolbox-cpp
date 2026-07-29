#include "numerical/analysis/Decibels.hpp"

namespace analysis
{
    template float ToDecibels<float>(float);
    template float FromDecibels<float>(float);
    template float AttenuationDb<float>(float, float);
    template float RippleDb<float>(float, float);
}

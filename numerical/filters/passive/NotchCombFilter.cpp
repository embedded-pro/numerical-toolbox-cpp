#include "numerical/filters/passive/NotchCombFilter.hpp"

namespace filters::passive
{
    template class NotchFilter<float>;
    template class CombFilter<float, 20, false>;
    template class CombFilter<float, 20, true>;
}

#include "numerical/filters/passive/SavitzkyGolayFilter.hpp"

namespace filters::passive
{
    template class SavitzkyGolayFilter<float, 5, 2, 0>;
    template class SavitzkyGolayFilter<float, 5, 2, 1>;
}

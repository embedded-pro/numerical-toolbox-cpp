#include "numerical/filters/passive/MovingAverage.hpp"

namespace filters::passive
{
    template class MovingAverage<float, 1>;
    template class MovingAverage<float, 4>;
}

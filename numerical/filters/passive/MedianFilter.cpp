#include "numerical/filters/passive/MedianFilter.hpp"

namespace filters::passive
{
    template class MedianFilter<float, 3>;
    template class MedianFilter<float, 5>;
}

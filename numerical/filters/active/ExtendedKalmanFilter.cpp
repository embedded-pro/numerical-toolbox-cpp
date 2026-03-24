#include "numerical/filters/active/ExtendedKalmanFilter.hpp"

namespace filters
{
    template class ExtendedKalmanFilter<float, 2, 1, 0>;
    template class ExtendedKalmanFilter<float, 2, 1, 1>;
    template class ExtendedKalmanFilter<float, 3, 1, 0>;
}

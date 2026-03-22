#include "numerical/filters/active/UnscentedKalmanFilter.hpp"

namespace filters
{
    template class UnscentedKalmanFilter<float, 2, 1, 0>;
    template class UnscentedKalmanFilter<float, 2, 1, 1>;
    template class UnscentedKalmanFilter<float, 3, 1, 0>;
}

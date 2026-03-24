#include "numerical/filters/active/KalmanFilter.hpp"

namespace filters
{
    template class KalmanFilter<float, 2, 1, 0>;
    template class KalmanFilter<float, 3, 1, 0>;
    template class KalmanFilter<float, 4, 2, 0>;
    template class KalmanFilter<float, 2, 1, 1>;
}

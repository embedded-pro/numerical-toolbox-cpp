#include "numerical/filters/active/SquareRootKalmanFilter.hpp"

namespace filters
{
    template class SquareRootKalmanFilter<float, 2, 1, 0>;
    template class SquareRootKalmanFilter<float, 2, 1, 1>;
    template class SquareRootKalmanFilter<float, 3, 1, 0>;
}

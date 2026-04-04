#include "numerical/filters/active/KalmanSmoother.hpp"

namespace filters
{
    template class KalmanSmoother<2, 1, 10>;
    template class KalmanSmoother<4, 2, 20>;
}

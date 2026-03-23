#include "numerical/filters/active/KalmanFilterBase.hpp"

namespace filters
{
    template class KalmanFilterBase<float, 2, 1, 0>;
    template class KalmanFilterBase<float, 2, 1, 1>;
    template class KalmanFilterBase<float, 3, 1, 0>;
    template class KalmanFilterBase<float, 4, 2, 0>;
}

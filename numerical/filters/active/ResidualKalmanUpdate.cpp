#include "numerical/filters/active/ResidualKalmanUpdate.hpp"

namespace filters
{
    template class ResidualKalmanUpdate<float, 1, 1>;
    template class ResidualKalmanUpdate<float, 2, 1>;
    template class ResidualKalmanUpdate<float, 2, 2>;
    template class ResidualKalmanUpdate<float, 3, 2>;
    template class ResidualKalmanUpdate<float, 4, 1>;
}

#include "numerical/dynamics/EulerLagrangeSolver.hpp"

namespace dynamics
{
    template class EulerLagrangeSolver<float, 1>;
    template class EulerLagrangeSolver<float, 2>;
    template class EulerLagrangeSolver<float, 3>;
}

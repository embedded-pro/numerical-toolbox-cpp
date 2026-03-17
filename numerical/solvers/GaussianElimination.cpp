#include "numerical/solvers/GaussianElimination.hpp"

namespace solvers
{
    template class GaussianElimination<float, 3>;

    template class GaussianElimination<math::Q15, 3>;

    template class GaussianElimination<math::Q31, 3>;
}

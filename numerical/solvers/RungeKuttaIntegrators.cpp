#include "numerical/solvers/RungeKuttaIntegrators.hpp"

namespace solvers
{
    template class RungeKutta4<float, 1, 0>;
    template class DormandPrince45<float, 1, 0>;
}

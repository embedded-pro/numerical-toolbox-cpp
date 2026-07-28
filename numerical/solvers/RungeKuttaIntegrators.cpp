#include "numerical/solvers/RungeKuttaIntegrators.hpp"
#include "numerical/solvers/DormandPrince45.hpp"

namespace solvers
{
    template class RungeKutta4<float, 1, 0>;
    template class DormandPrince45<float, 1, 0>;
}

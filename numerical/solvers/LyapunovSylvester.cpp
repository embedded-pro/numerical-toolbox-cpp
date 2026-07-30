#include "numerical/solvers/LyapunovSylvester.hpp"

namespace solvers
{
    template class SylvesterSolver<float, 2, 2>;
    template class SylvesterSolver<float, 3, 3>;
}

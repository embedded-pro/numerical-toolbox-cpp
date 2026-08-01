#include "numerical/solvers/JacobiEigenSolver.hpp"

namespace solvers
{
    template class JacobiEigenSolver<float, 2>;
    template class JacobiEigenSolver<float, 3>;
    template class JacobiEigenSolver<float, 4>;
}

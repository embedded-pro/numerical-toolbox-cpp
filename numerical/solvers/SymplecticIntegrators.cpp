#include "numerical/solvers/SymplecticIntegrators.hpp"

namespace solvers
{
    template class SemiImplicitEuler<float, 1, 1>;
    template class VelocityVerlet<float, 1, 1>;
}

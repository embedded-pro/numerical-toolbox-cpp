#include "numerical/solvers/ActiveSetQp.hpp"

namespace solvers
{
    template class ActiveSetQp<float, 2, 1, 2>;
    template class ActiveSetQp<float, 2, 0, 4>;
    template class ActiveSetQp<float, 4, 2, 8>;
}

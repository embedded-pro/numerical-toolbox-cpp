#include "numerical/solvers/ConditionNumber.hpp"

namespace solvers
{
    template std::optional<float> ConditionNumber<float, 2>(const math::SquareMatrix<float, 2>&);
}

#include "numerical/optimization/LevenbergMarquardt.hpp"

namespace optimization
{
    template class ConstantDamping<float, 2, 2>;
    template class LevenbergMarquardt<float, math::Vector<float, 2>, 2, 2>;
}

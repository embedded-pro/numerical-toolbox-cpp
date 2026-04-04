#include "numerical/optimization/BayesianOptimization.hpp"

namespace optimization
{
    template class BayesianOptimization<1, 10, 50>;
    template class BayesianOptimization<2, 30, 200>;
}

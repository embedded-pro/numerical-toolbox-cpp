#include "numerical/estimators/YuleWalker.hpp"

namespace estimators
{
    template class YuleWalker<float, 256, 2>;
    template class YuleWalker<float, 128, 4>;
}

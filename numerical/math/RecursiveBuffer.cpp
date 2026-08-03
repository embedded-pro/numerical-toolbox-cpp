#include "numerical/math/QNumber.hpp"
#include "numerical/math/RecursiveBuffer.hpp"

namespace math
{
    template class RecursiveBuffer<float, 4>;
    template class RecursiveBuffer<Q31, 4>;
    template class RecursiveBuffer<Q15, 4>;
}

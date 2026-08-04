#include "numerical/math/QNumber.hpp"
#include "numerical/math/Toeplitz.hpp"

namespace math
{
    template class ToeplitzMatrix<float, 2>;
    template class ToeplitzMatrix<Q15, 2>;
    template class ToeplitzMatrix<Q31, 2>;
}

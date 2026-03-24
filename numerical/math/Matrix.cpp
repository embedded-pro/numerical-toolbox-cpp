#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"

namespace math
{
    template class Matrix<float, 2, 2>;
    template class Matrix<Q15, 2, 2>;
    template class Matrix<Q31, 2, 2>;
    template class Matrix<float, 3, 3>;
}

#include "numerical/math/MatrixNorms.hpp"

namespace math
{
    template float FrobeniusNorm<float, 2, 2>(const Matrix<float, 2, 2>&);
    template float OneNorm<float, 2, 2>(const Matrix<float, 2, 2>&);
    template float InfinityNorm<float, 2, 2>(const Matrix<float, 2, 2>&);
    template float VectorNorm<float, 2>(const Vector<float, 2>&);
    template std::optional<Vector<float, 2>> Normalize<float, 2>(const Vector<float, 2>&);
}

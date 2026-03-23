#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cmath>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace solvers
{
    // Precondition: A must be symmetric positive-definite
    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED math::SquareMatrix<T, N> CholeskyDecomposition(const math::SquareMatrix<T, N>& A)
    {
        math::SquareMatrix<T, N> L;

        for (std::size_t i = 0; i < N; ++i)
        {
            for (std::size_t j = 0; j <= i; ++j)
            {
                float sum = 0.0f;

                for (std::size_t k = 0; k < j; ++k)
                    sum += math::ToFloat(L.at(i, k)) * math::ToFloat(L.at(j, k));

                if (i == j)
                    L.at(i, j) = T(std::sqrt(math::ToFloat(A.at(i, i)) - sum));
                else
                    L.at(i, j) = T((math::ToFloat(A.at(i, j)) - sum) / math::ToFloat(L.at(j, j)));
            }
        }

        return L;
    }
}

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED SquareMatrix<T, N> Symmetrize(const SquareMatrix<T, N>& m)
    {
        static_assert(std::is_floating_point_v<T>, "Symmetrize supports floating-point types");
        return (m + m.Transpose()) * T(0.5f);
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    [[nodiscard]] OPTIMIZE_FOR_SPEED SquareMatrix<T, Rows> CongruenceTransform(const Matrix<T, Rows, Cols>& a, const SquareMatrix<T, Cols>& m)
    {
        static_assert(std::is_floating_point_v<T>, "CongruenceTransform supports floating-point types");
        return a * m * a.Transpose();
    }
}

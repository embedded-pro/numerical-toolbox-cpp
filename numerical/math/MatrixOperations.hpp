#pragma once

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

    template<typename T, std::size_t N>
    [[nodiscard]] SquareMatrix<T, N> Diagonal(const Vector<T, N>& d)
    {
        static_assert(std::is_floating_point_v<T>, "Diagonal supports floating-point types");
        SquareMatrix<T, N> result{};
        for (std::size_t i = 0; i < N; ++i)
            result.at(i, i) = d.at(i, 0);
        return result;
    }

    template<typename T, std::size_t N>
    [[nodiscard]] Vector<T, N> DiagonalOf(const SquareMatrix<T, N>& m)
    {
        static_assert(std::is_floating_point_v<T>, "DiagonalOf supports floating-point types");
        Vector<T, N> result{};
        for (std::size_t i = 0; i < N; ++i)
            result.at(i, 0) = m.at(i, i);
        return result;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    [[nodiscard]] OPTIMIZE_FOR_SPEED Matrix<T, Rows, Cols> ScaleColumns(const Matrix<T, Rows, Cols>& m, const Vector<T, Cols>& d)
    {
        static_assert(std::is_floating_point_v<T>, "ScaleColumns supports floating-point types");
        Matrix<T, Rows, Cols> result{ m };
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                result.at(i, j) *= d.at(j, 0);
        return result;
    }

    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED SquareMatrix<T, N> DiagonalCongruence(const Vector<T, N>& d, const SquareMatrix<T, N>& m)
    {
        static_assert(std::is_floating_point_v<T>, "DiagonalCongruence supports floating-point types");
        SquareMatrix<T, N> result{ m };
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = 0; j < N; ++j)
                result.at(i, j) *= d.at(i, 0) * d.at(j, 0);
        return result;
    }
}

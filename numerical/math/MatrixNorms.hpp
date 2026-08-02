#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cmath>
#include <cstddef>
#include <optional>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t Rows, std::size_t Cols>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T FrobeniusNorm(const Matrix<T, Rows, Cols>& a)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T sum{};
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                sum += a.at(i, j) * a.at(i, j);
        return std::sqrt(sum);
    }

    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T OffDiagonalFrobeniusNorm(const Matrix<T, N, N>& a)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T sum{};
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = i + 1; j < N; ++j)
                sum += a.at(i, j) * a.at(i, j) + a.at(j, i) * a.at(j, i);
        return std::sqrt(sum);
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T OneNorm(const Matrix<T, Rows, Cols>& a)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T maxColSum{};
        for (std::size_t j = 0; j < Cols; ++j)
        {
            T colSum{};
            for (std::size_t i = 0; i < Rows; ++i)
                colSum += std::abs(a.at(i, j));
            if (colSum > maxColSum)
                maxColSum = colSum;
        }
        return maxColSum;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T InfinityNorm(const Matrix<T, Rows, Cols>& a)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T maxRowSum{};
        for (std::size_t i = 0; i < Rows; ++i)
        {
            T rowSum{};
            for (std::size_t j = 0; j < Cols; ++j)
                rowSum += std::abs(a.at(i, j));
            if (rowSum > maxRowSum)
                maxRowSum = rowSum;
        }
        return maxRowSum;
    }

    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T DotProduct(const Vector<T, Size>& a, const Vector<T, Size>& b)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T sum{};
        for (std::size_t i = 0; i < Size; ++i)
            sum += a.at(i, 0) * b.at(i, 0);
        return sum;
    }

    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T VectorNorm(const Vector<T, Size>& v)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        return std::sqrt(DotProduct(v, v));
    }

    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED std::optional<Vector<T, Size>> Normalize(const Vector<T, Size>& v)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T n = VectorNorm(v);
        if (n < static_cast<T>(1e-12))
            return std::nullopt;
        Vector<T, Size> result;
        for (std::size_t i = 0; i < Size; ++i)
            result.at(i, 0) = v.at(i, 0) / n;
        return result;
    }
}

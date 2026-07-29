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
    [[nodiscard]] OPTIMIZE_FOR_SPEED T VectorNorm(const Vector<T, Size>& v)
    {
        static_assert(std::is_floating_point_v<T>, "MatrixNorms supports floating-point types");
        T sum{};
        for (std::size_t i = 0; i < Size; ++i)
            sum += v.at(i, 0) * v.at(i, 0);
        return std::sqrt(sum);
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

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template float FrobeniusNorm<float, 2, 2>(const Matrix<float, 2, 2>&);
    extern template float OneNorm<float, 2, 2>(const Matrix<float, 2, 2>&);
    extern template float InfinityNorm<float, 2, 2>(const Matrix<float, 2, 2>&);
    extern template float VectorNorm<float, 2>(const Vector<float, 2>&);
    extern template std::optional<Vector<float, 2>> Normalize<float, 2>(const Vector<float, 2>&);
#endif
}

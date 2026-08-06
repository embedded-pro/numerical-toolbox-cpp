#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/Math.hpp"
#include <cstddef>

namespace math
{
    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED std::size_t FindPartialPivotRow(const Matrix<T, N, N>& matrix, std::size_t col)
    {
        std::size_t pivotRow = col;
        float maxVal = math::Abs(ToFloat(matrix.at(col, col)));

        for (std::size_t row = col + 1; row < N; ++row)
        {
            float absVal = math::Abs(ToFloat(matrix.at(row, col)));
            if (absVal > maxVal)
            {
                maxVal = absVal;
                pivotRow = row;
            }
        }

        return pivotRow;
    }

    namespace detail
    {
        template<bool Unit, typename T, std::size_t N>
        [[nodiscard]] OPTIMIZE_FOR_SPEED Vector<T, N> ForwardSubstitute(const Matrix<T, N, N>& l, const Vector<T, N>& c)
        {
            Vector<T, N> x{};

            for (std::size_t i = 0; i < N; ++i)
            {
                T sum = c.at(i, 0);
                for (std::size_t j = 0; j < i; ++j)
                    sum = sum - l.at(i, j) * x.at(j, 0);

                if constexpr (Unit)
                {
                    x.at(i, 0) = sum;
                }
                else
                {
                    really_assert(math::Abs(ToFloat(l.at(i, i))) > 0.0f);
                    x.at(i, 0) = sum / l.at(i, i);
                }
            }

            return x;
        }
    }

    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED Vector<T, N> SolveUnitLowerTriangular(const Matrix<T, N, N>& l, const Vector<T, N>& c)
    {
        return detail::ForwardSubstitute<true>(l, c);
    }

    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED Vector<T, N> SolveLowerTriangular(const Matrix<T, N, N>& l, const Vector<T, N>& c)
    {
        return detail::ForwardSubstitute<false>(l, c);
    }

    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED Vector<T, N> SolveUpperTriangular(const Matrix<T, N, N>& r, const Vector<T, N>& c)
    {
        Vector<T, N> x{};

        for (std::size_t i = N; i > 0; --i)
        {
            std::size_t row = i - 1;
            T sum = c.at(row, 0);
            for (std::size_t j = row + 1; j < N; ++j)
                sum = sum - r.at(row, j) * x.at(j, 0);

            really_assert(math::Abs(ToFloat(r.at(row, row))) > 0.0f);
            x.at(row, 0) = sum / r.at(row, row);
        }

        return x;
    }
}

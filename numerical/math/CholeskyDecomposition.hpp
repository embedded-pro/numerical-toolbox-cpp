#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <cstddef>
#include <optional>

namespace math
{
    namespace detail
    {
        template<typename T, std::size_t N>
        [[nodiscard]] OPTIMIZE_FOR_SPEED constexpr float CholeskyInnerProduct(const SquareMatrix<T, N>& l, std::size_t i, std::size_t j)
        {
            float sum = 0.0f;
            for (std::size_t k = 0; k < j; ++k)
                sum += ToFloat(l.at(i, k)) * ToFloat(l.at(j, k));
            return sum;
        }
    }

    template<typename T, std::size_t N>
    class CholeskyDecomposition
    {
        static_assert(detail::is_supported_type_v<T>,
            "CholeskyDecomposition only supports float or QNumber types");

    public:
        [[nodiscard]] static OPTIMIZE_FOR_SPEED constexpr SquareMatrix<T, N> Factor(const SquareMatrix<T, N>& a);
        [[nodiscard]] static OPTIMIZE_FOR_SPEED constexpr std::optional<SquareMatrix<T, N>> TryFactor(const SquareMatrix<T, N>& a);
        [[nodiscard]] static OPTIMIZE_FOR_SPEED constexpr std::optional<Vector<T, N>> Solve(const SquareMatrix<T, N>& a, const Vector<T, N>& b);
    };

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr std::optional<SquareMatrix<T, N>> CholeskyDecomposition<T, N>::TryFactor(const SquareMatrix<T, N>& a)
    {
        SquareMatrix<T, N> l{};

        for (std::size_t i = 0; i < N; ++i)
        {
            for (std::size_t j = 0; j <= i; ++j)
            {
                const float sum = ToFloat(a.at(i, j)) - detail::CholeskyInnerProduct(l, i, j);

                if (i != j)
                    l.at(i, j) = T(sum / ToFloat(l.at(j, j)));
                else if (sum < 1e-10f)
                    return std::nullopt;
                else
                    l.at(i, j) = T(math::Sqrt(sum));
            }
        }

        return l;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr SquareMatrix<T, N> CholeskyDecomposition<T, N>::Factor(const SquareMatrix<T, N>& a)
    {
        return TryFactor(a).value_or(SquareMatrix<T, N>{});
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr std::optional<Vector<T, N>> CholeskyDecomposition<T, N>::Solve(const SquareMatrix<T, N>& a, const Vector<T, N>& b)
    {
        auto l = TryFactor(a);
        if (!l.has_value())
            return std::nullopt;

        const Vector<T, N> y = SolveLowerTriangular(l.value(), b);
        return SolveUpperTriangular(l.value().Transpose(), y);
    }
}

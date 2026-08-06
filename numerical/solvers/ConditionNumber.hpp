#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace solvers
{
    namespace detail
    {
        template<typename T, std::size_t N>
        bool HasZeroRow(const math::SquareMatrix<T, N>& a)
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                bool allZero = true;
                for (std::size_t j = 0; j < N; ++j)
                {
                    if (math::Abs(a.at(i, j)) > static_cast<T>(1e-12))
                    {
                        allZero = false;
                        break;
                    }
                }
                if (allZero)
                    return true;
            }
            return false;
        }
    }

    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED std::optional<T> ConditionNumber(const math::SquareMatrix<T, N>& a)
    {
        static_assert(std::is_floating_point_v<T>, "ConditionNumber supports floating-point types");
        if (detail::HasZeroRow(a))
            return std::nullopt;
        T normA = math::OneNorm(a);
        if (normA < static_cast<T>(1e-12))
            return std::nullopt;
        auto invA = SolveSystem(a, math::SquareMatrix<T, N>::Identity());
        return normA * math::OneNorm(invA);
    }
}

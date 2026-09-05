#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED std::optional<T> ConditionNumber(const math::SquareMatrix<T, N>& a)
    {
        static_assert(std::is_floating_point_v<T>, "ConditionNumber supports floating-point types");

        const T normA = math::OneNorm(a);

        if (!math::IsFinite(normA) || normA <= T{})
            return std::nullopt;

        const auto invA = TrySolveSystem(a, math::SquareMatrix<T, N>::Identity());

        if (!invA)
            return std::nullopt;

        const T normInv = math::OneNorm(*invA);

        if (!math::IsFinite(normInv))
            return std::nullopt;

        return normA * normInv;
    }
}

#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t MaxSize>
    class TridiagonalSolver
    {
        static_assert(std::is_floating_point_v<T>, "TridiagonalSolver supports floating-point types only");
        static_assert(MaxSize >= 1, "TridiagonalSolver requires MaxSize >= 1");

    public:
        using Band = std::array<T, MaxSize>;

        bool Factor(const Band& subDiagonal, const Band& diagonal, const Band& superDiagonal, std::size_t n);
        OPTIMIZE_FOR_SPEED Band Solve(const Band& rhs) const;
        std::size_t Size() const;

    private:
        static T Scale(const Band& subDiagonal, const Band& diagonal, const Band& superDiagonal, std::size_t n);

        Band sub{};
        Band invPivot{};
        Band supPrime{};
        std::size_t size{ 0 };
    };

    template<typename T, std::size_t MaxSize>
    bool TridiagonalSolver<T, MaxSize>::Factor(const Band& subDiagonal, const Band& diagonal, const Band& superDiagonal, std::size_t n)
    {
        size = 0;
        if (n == 0 || n > MaxSize)
            return false;

        const T threshold{ std::numeric_limits<T>::epsilon() * Scale(subDiagonal, diagonal, superDiagonal, n) };

        for (std::size_t k = 0; k < n; ++k)
        {
            const T pivot{ k == 0 ? diagonal[0] : diagonal[k] - subDiagonal[k] * supPrime[k - 1] };
            if (math::Abs(pivot) <= threshold)
                return false;

            invPivot[k] = T{ 1 } / pivot;
            supPrime[k] = superDiagonal[k] * invPivot[k];
        }

        sub = subDiagonal;
        size = n;
        return true;
    }

    template<typename T, std::size_t MaxSize>
    OPTIMIZE_FOR_SPEED auto TridiagonalSolver<T, MaxSize>::Solve(const Band& rhs) const -> Band
    {
        really_assert(size > 0);

        Band x{};
        x[0] = rhs[0] * invPivot[0];
        for (std::size_t k = 1; k < size; ++k)
            x[k] = (rhs[k] - sub[k] * x[k - 1]) * invPivot[k];

        for (std::size_t k = size - 1; k > 0; --k)
            x[k - 1] -= supPrime[k - 1] * x[k];

        return x;
    }

    template<typename T, std::size_t MaxSize>
    std::size_t TridiagonalSolver<T, MaxSize>::Size() const
    {
        return size;
    }

    template<typename T, std::size_t MaxSize>
    T TridiagonalSolver<T, MaxSize>::Scale(const Band& subDiagonal, const Band& diagonal, const Band& superDiagonal, std::size_t n)
    {
        T scale{};
        for (std::size_t k = 0; k < n; ++k)
        {
            scale = std::max(scale, math::Abs(diagonal[k]));
            if (k > 0)
                scale = std::max(scale, math::Abs(subDiagonal[k]));
            if (k + 1 < n)
                scale = std::max(scale, math::Abs(superDiagonal[k]));
        }

        return scale;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class TridiagonalSolver<float, 8>;
#endif
}

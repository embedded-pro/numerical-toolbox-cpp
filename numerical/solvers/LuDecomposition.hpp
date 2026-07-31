#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t N>
    class LuDecomposition
    {
        static_assert(std::is_floating_point_v<T>, "LuDecomposition supports floating-point types only");

    public:
        LuDecomposition() = default;

        OPTIMIZE_FOR_SPEED bool Decompose(const math::Matrix<T, N, N>& a);
        OPTIMIZE_FOR_SPEED math::Vector<T, N> Solve(const math::Vector<T, N>& b) const;
        math::Matrix<T, N, N> Inverse() const;
        T Determinant() const;

        bool IsSingular() const
        {
            return singular;
        }

        math::Matrix<T, N, N> L() const;
        math::Matrix<T, N, N> U() const;
        math::Matrix<T, N, N> P() const;

    private:
        math::Matrix<T, N, N> lu{};
        std::array<std::size_t, N> piv{};
        int pivotSign{ 1 };
        bool singular{ false };
    };

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED bool LuDecomposition<T, N>::Decompose(const math::Matrix<T, N, N>& a)
    {
        lu = a;
        for (std::size_t i = 0; i < N; ++i)
            piv[i] = i;
        pivotSign = 1;
        singular = false;

        constexpr T relativeEps{ T{ 1e-6f } };
        T maxPivot{ T{} };

        for (std::size_t k = 0; k < N; ++k)
        {
            std::size_t p = k;
            T maxVal = std::abs(lu.at(k, k));
            for (std::size_t i = k + 1; i < N; ++i)
            {
                T candidate = std::abs(lu.at(i, k));
                if (candidate > maxVal)
                {
                    maxVal = candidate;
                    p = i;
                }
            }

            if (maxVal > maxPivot)
                maxPivot = maxVal;

            if (maxVal <= maxPivot * relativeEps)
            {
                singular = true;
                return false;
            }

            if (p != k)
            {
                for (std::size_t j = 0; j < N; ++j)
                {
                    T tmp = lu.at(p, j);
                    lu.at(p, j) = lu.at(k, j);
                    lu.at(k, j) = tmp;
                }
                std::size_t tmpIdx = piv[p];
                piv[p] = piv[k];
                piv[k] = tmpIdx;
                pivotSign = -pivotSign;
            }

            for (std::size_t i = k + 1; i < N; ++i)
            {
                lu.at(i, k) /= lu.at(k, k);
                for (std::size_t j = k + 1; j < N; ++j)
                    lu.at(i, j) -= lu.at(i, k) * lu.at(k, j);
            }
        }

        return true;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED math::Vector<T, N> LuDecomposition<T, N>::Solve(const math::Vector<T, N>& b) const
    {
        math::Vector<T, N> pb{};
        for (std::size_t i = 0; i < N; ++i)
            pb.at(i, 0) = b.at(piv[i], 0);

        math::Vector<T, N> y = math::SolveUnitLowerTriangular(lu, pb);
        return math::SolveUpperTriangular(lu, y);
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, N> LuDecomposition<T, N>::Inverse() const
    {
        math::Matrix<T, N, N> result{};
        for (std::size_t j = 0; j < N; ++j)
        {
            math::Vector<T, N> e{};
            e.at(j, 0) = T{ 1 };
            auto col = Solve(e);
            for (std::size_t i = 0; i < N; ++i)
                result.at(i, j) = col.at(i, 0);
        }
        return result;
    }

    template<typename T, std::size_t N>
    T LuDecomposition<T, N>::Determinant() const
    {
        T det{ static_cast<T>(pivotSign) };
        for (std::size_t k = 0; k < N; ++k)
            det *= lu.at(k, k);
        return det;
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, N> LuDecomposition<T, N>::L() const
    {
        math::Matrix<T, N, N> result{};
        for (std::size_t i = 0; i < N; ++i)
        {
            result.at(i, i) = T{ 1 };
            for (std::size_t j = 0; j < i; ++j)
                result.at(i, j) = lu.at(i, j);
        }
        return result;
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, N> LuDecomposition<T, N>::U() const
    {
        math::Matrix<T, N, N> result{};
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t j = i; j < N; ++j)
                result.at(i, j) = lu.at(i, j);
        return result;
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, N> LuDecomposition<T, N>::P() const
    {
        math::Matrix<T, N, N> result{};
        for (std::size_t i = 0; i < N; ++i)
            result.at(i, piv[i]) = T{ 1 };
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LuDecomposition<float, 3>;
    extern template class LuDecomposition<float, 4>;
#endif
}

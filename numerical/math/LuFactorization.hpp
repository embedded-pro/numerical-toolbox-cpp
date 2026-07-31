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

namespace math
{
    template<typename T, std::size_t N>
    class LuFactorization
    {
        static_assert(std::is_floating_point_v<T>, "LuFactorization supports floating-point types only");

    public:
        LuFactorization() = default;

        OPTIMIZE_FOR_SPEED bool Decompose(const Matrix<T, N, N>& a);
        OPTIMIZE_FOR_SPEED Vector<T, N> Solve(const Vector<T, N>& b) const;
        Matrix<T, N, N> Inverse() const;
        T Determinant() const;
        bool IsSingular() const;
        const Matrix<T, N, N>& Packed() const;
        std::size_t PivotRow(std::size_t i) const;
        int PivotSign() const;

    private:
        Matrix<T, N, N> lu{};
        std::array<std::size_t, N> piv{};
        int pivotSign{ 1 };
        bool singular{ false };
    };

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED bool LuFactorization<T, N>::Decompose(const Matrix<T, N, N>& a)
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
    OPTIMIZE_FOR_SPEED Vector<T, N> LuFactorization<T, N>::Solve(const Vector<T, N>& b) const
    {
        Vector<T, N> pb{};
        for (std::size_t i = 0; i < N; ++i)
            pb.at(i, 0) = b.at(piv[i], 0);

        Vector<T, N> y = SolveUnitLowerTriangular(lu, pb);
        return SolveUpperTriangular(lu, y);
    }

    template<typename T, std::size_t N>
    Matrix<T, N, N> LuFactorization<T, N>::Inverse() const
    {
        Matrix<T, N, N> result{};
        for (std::size_t j = 0; j < N; ++j)
        {
            Vector<T, N> e{};
            e.at(j, 0) = T{ 1 };
            auto col = Solve(e);
            for (std::size_t i = 0; i < N; ++i)
                result.at(i, j) = col.at(i, 0);
        }
        return result;
    }

    template<typename T, std::size_t N>
    T LuFactorization<T, N>::Determinant() const
    {
        T det{ static_cast<T>(pivotSign) };
        for (std::size_t k = 0; k < N; ++k)
            det *= lu.at(k, k);
        return det;
    }

    template<typename T, std::size_t N>
    bool LuFactorization<T, N>::IsSingular() const
    {
        return singular;
    }

    template<typename T, std::size_t N>
    const Matrix<T, N, N>& LuFactorization<T, N>::Packed() const
    {
        return lu;
    }

    template<typename T, std::size_t N>
    std::size_t LuFactorization<T, N>::PivotRow(std::size_t i) const
    {
        return piv[i];
    }

    template<typename T, std::size_t N>
    int LuFactorization<T, N>::PivotSign() const
    {
        return pivotSign;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LuFactorization<float, 2>;
    extern template class LuFactorization<float, 3>;
    extern template class LuFactorization<float, 4>;
#endif
}

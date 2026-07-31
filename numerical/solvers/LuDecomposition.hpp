#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/LuFactorization.hpp"
#include "numerical/math/Matrix.hpp"
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

        bool Decompose(const math::Matrix<T, N, N>& a);
        math::Vector<T, N> Solve(const math::Vector<T, N>& b) const;
        math::Matrix<T, N, N> Inverse() const;
        T Determinant() const;
        bool IsSingular() const;
        math::Matrix<T, N, N> L() const;
        math::Matrix<T, N, N> U() const;
        math::Matrix<T, N, N> P() const;

    private:
        math::LuFactorization<T, N> factorization{};
    };

    template<typename T, std::size_t N>
    bool LuDecomposition<T, N>::Decompose(const math::Matrix<T, N, N>& a)
    {
        return factorization.Decompose(a);
    }

    template<typename T, std::size_t N>
    math::Vector<T, N> LuDecomposition<T, N>::Solve(const math::Vector<T, N>& b) const
    {
        return factorization.Solve(b);
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, N> LuDecomposition<T, N>::Inverse() const
    {
        return factorization.Inverse();
    }

    template<typename T, std::size_t N>
    T LuDecomposition<T, N>::Determinant() const
    {
        return factorization.Determinant();
    }

    template<typename T, std::size_t N>
    bool LuDecomposition<T, N>::IsSingular() const
    {
        return factorization.IsSingular();
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, N> LuDecomposition<T, N>::L() const
    {
        const auto& lu = factorization.Packed();
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
        const auto& lu = factorization.Packed();
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
            result.at(i, factorization.PivotRow(i)) = T{ 1 };
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LuDecomposition<float, 3>;
    extern template class LuDecomposition<float, 4>;
#endif
}

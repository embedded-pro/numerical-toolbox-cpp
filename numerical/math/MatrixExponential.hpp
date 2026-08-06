#pragma once
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <array>
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t N>
    class MatrixExponential
    {
        static_assert(std::is_floating_point_v<T>, "MatrixExponential supports floating-point types");

    public:
        MatrixExponential() = default;

        OPTIMIZE_FOR_SPEED SquareMatrix<T, N> Compute(const SquareMatrix<T, N>& a);
        OPTIMIZE_FOR_SPEED SquareMatrix<T, N> Compute(const SquareMatrix<T, N>& a, T dt);

    private:
        static constexpr T c0{ T{ 1 } };
        static constexpr T c1{ T{ 1 } / T{ 2 } };
        static constexpr T c2{ T{ 5 } / T{ 44 } };
        static constexpr T c3{ T{ 1 } / T{ 66 } };
        static constexpr T c4{ T{ 1 } / T{ 792 } };
        static constexpr T c5{ T{ 1 } / T{ 15840 } };
        static constexpr T c6{ T{ 1 } / T{ 665280 } };

        void PadeNumeratorDenominator(const SquareMatrix<T, N>& as,
            SquareMatrix<T, N>& num,
            SquareMatrix<T, N>& den);

        SquareMatrix<T, N> SolvePade(const SquareMatrix<T, N>& den, const SquareMatrix<T, N>& num);
    };

    template<typename T, std::size_t N>
    void MatrixExponential<T, N>::PadeNumeratorDenominator(const SquareMatrix<T, N>& as,
        SquareMatrix<T, N>& num,
        SquareMatrix<T, N>& den)
    {
        const auto identity = SquareMatrix<T, N>::Identity();
        const auto a2 = as * as;
        const auto a4 = a2 * a2;
        const auto a6 = a4 * a2;

        const auto vEven = identity * c0 + a2 * c2 + a4 * c4 + a6 * c6;
        const auto uOdd = as * (identity * c1 + a2 * c3 + a4 * c5);

        num = vEven + uOdd;
        den = vEven - uOdd;
    }

    template<typename T, std::size_t N>
    SquareMatrix<T, N> MatrixExponential<T, N>::SolvePade(const SquareMatrix<T, N>& den, const SquareMatrix<T, N>& num)
    {
        SquareMatrix<T, N> lu = den;
        std::array<std::size_t, N> piv{};
        for (std::size_t i = 0; i < N; ++i)
            piv[i] = i;

        for (std::size_t k = 0; k < N; ++k)
        {
            std::size_t p = k;
            T maxVal = math::Abs(lu.at(k, k));
            for (std::size_t i = k + 1; i < N; ++i)
            {
                T candidate = math::Abs(lu.at(i, k));
                if (candidate > maxVal)
                {
                    maxVal = candidate;
                    p = i;
                }
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
            }

            for (std::size_t i = k + 1; i < N; ++i)
            {
                lu.at(i, k) /= lu.at(k, k);
                for (std::size_t j = k + 1; j < N; ++j)
                    lu.at(i, j) -= lu.at(i, k) * lu.at(k, j);
            }
        }

        SquareMatrix<T, N> result{};
        for (std::size_t col = 0; col < N; ++col)
        {
            Vector<T, N> b{};
            for (std::size_t i = 0; i < N; ++i)
                b.at(i, 0) = num.at(piv[i], col);

            Vector<T, N> y = SolveUnitLowerTriangular(lu, b);
            Vector<T, N> x = SolveUpperTriangular(lu, y);

            for (std::size_t i = 0; i < N; ++i)
                result.at(i, col) = x.at(i, 0);
        }

        return result;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED SquareMatrix<T, N> MatrixExponential<T, N>::Compute(const SquareMatrix<T, N>& a)
    {
        const T norm = InfinityNorm(a);
        int s{ 0 };
        if (norm > T{ 1 })
        {
            const T lg = math::Log2(norm);
            s = static_cast<int>(math::Ceil(lg));
            if (s < 0)
                s = 0;
        }

        const T scale = T{ 1 } / static_cast<T>(1 << s);
        const auto as = a * scale;

        SquareMatrix<T, N> num{};
        SquareMatrix<T, N> den{};
        PadeNumeratorDenominator(as, num, den);

        auto r = SolvePade(den, num);

        for (int k = 0; k < s; ++k)
            r = r * r;

        return r;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED SquareMatrix<T, N> MatrixExponential<T, N>::Compute(const SquareMatrix<T, N>& a, T dt)
    {
        const auto adt = a * dt;
        return Compute(adt);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class MatrixExponential<float, 2>;
    extern template class MatrixExponential<float, 3>;
#endif
}

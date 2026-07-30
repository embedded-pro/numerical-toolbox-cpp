#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t N, std::size_t M>
    class SylvesterSolver
    {
        static_assert(std::is_floating_point_v<T>, "SylvesterSolver supports floating-point types only");
        static_assert(N > 0 && M > 0, "SylvesterSolver requires positive dimensions");

        static constexpr std::size_t NM = N * M;

    public:
        SylvesterSolver() = default;

        OPTIMIZE_FOR_SPEED bool SolveSylvester(
            const math::SquareMatrix<T, N>& a,
            const math::SquareMatrix<T, M>& b,
            const math::Matrix<T, N, M>& c);

        OPTIMIZE_FOR_SPEED bool SolveContinuousLyapunov(
            const math::SquareMatrix<T, N>& a,
            const math::SquareMatrix<T, N>& q);

        OPTIMIZE_FOR_SPEED bool SolveDiscreteLyapunov(
            const math::SquareMatrix<T, N>& a,
            const math::SquareMatrix<T, N>& q);

        [[nodiscard]] const math::Matrix<T, N, M>& Solution() const
        {
            return solution;
        }

    private:
        static math::Matrix<T, N * M, 1> Vec(const math::Matrix<T, N, M>& x);
        static math::Matrix<T, N, M> UnVec(const math::Matrix<T, N * M, 1>& v);

        math::Matrix<T, N, M> solution{};
    };

    template<typename T, std::size_t N, std::size_t M>
    OPTIMIZE_FOR_SPEED bool SylvesterSolver<T, N, M>::SolveSylvester(
        const math::SquareMatrix<T, N>& a,
        const math::SquareMatrix<T, M>& b,
        const math::Matrix<T, N, M>& c)
    {
        math::Matrix<T, NM, NM> K{};
        for (std::size_t i = 0; i < M; ++i)
            for (std::size_t j = 0; j < M; ++j)
                for (std::size_t k = 0; k < N; ++k)
                    for (std::size_t l = 0; l < N; ++l)
                    {
                        T val{};
                        if (i == j)
                            val += a.at(k, l);
                        if (k == l)
                            val += b.at(j, i);
                        K.at(i * N + k, j * N + l) = val;
                    }

        math::Matrix<T, NM, 1> vecC = Vec(c);

        LuDecomposition<T, NM> lu{};
        if (!lu.Decompose(K))
            return false;

        solution = UnVec(lu.Solve(vecC));
        return true;
    }

    template<typename T, std::size_t N, std::size_t M>
    OPTIMIZE_FOR_SPEED bool SylvesterSolver<T, N, M>::SolveContinuousLyapunov(
        const math::SquareMatrix<T, N>& a,
        const math::SquareMatrix<T, N>& q)
    {
        static_assert(N == M, "SolveContinuousLyapunov requires square solver (N == M)");

        return SolveSylvester(a, a.Transpose(), q * T(-1));
    }

    template<typename T, std::size_t N, std::size_t M>
    OPTIMIZE_FOR_SPEED bool SylvesterSolver<T, N, M>::SolveDiscreteLyapunov(
        const math::SquareMatrix<T, N>& a,
        const math::SquareMatrix<T, N>& q)
    {
        static_assert(N == M, "SolveDiscreteLyapunov requires square solver (N == M)");

        math::Matrix<T, NM, NM> K{};
        for (std::size_t i = 0; i < N; ++i)
            for (std::size_t k = 0; k < N; ++k)
                for (std::size_t j = 0; j < N; ++j)
                    for (std::size_t l = 0; l < N; ++l)
                    {
                        T val = a.at(i, j) * a.at(k, l);
                        if (i == j && k == l)
                            val -= T(1);
                        K.at(i * N + k, j * N + l) = val;
                    }

        math::Matrix<T, NM, 1> rhs = Vec(q * T(-1));

        LuDecomposition<T, NM> lu{};
        if (!lu.Decompose(K))
            return false;

        solution = UnVec(lu.Solve(rhs));
        return true;
    }

    template<typename T, std::size_t N, std::size_t M>
    math::Matrix<T, N * M, 1> SylvesterSolver<T, N, M>::Vec(const math::Matrix<T, N, M>& x)
    {
        math::Matrix<T, NM, 1> v{};
        for (std::size_t i = 0; i < M; ++i)
            for (std::size_t k = 0; k < N; ++k)
                v.at(i * N + k, 0) = x.at(k, i);
        return v;
    }

    template<typename T, std::size_t N, std::size_t M>
    math::Matrix<T, N, M> SylvesterSolver<T, N, M>::UnVec(const math::Matrix<T, N * M, 1>& v)
    {
        math::Matrix<T, N, M> out{};
        for (std::size_t i = 0; i < M; ++i)
            for (std::size_t k = 0; k < N; ++k)
                out.at(k, i) = v.at(i * N + k, 0);
        return out;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SylvesterSolver<float, 2, 2>;
    extern template class SylvesterSolver<float, 3, 3>;
#endif
}

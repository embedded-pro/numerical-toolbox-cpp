#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t N>
    class JacobiEigenSolver
    {
        static_assert(std::is_floating_point_v<T>, "JacobiEigenSolver supports floating-point types only");
        static_assert(N >= 1, "JacobiEigenSolver requires N >= 1");

    public:
        JacobiEigenSolver() = default;

        OPTIMIZE_FOR_SPEED bool Solve(const math::Matrix<T, N, N>& a);

        const math::Vector<T, N>& Eigenvalues() const;
        const math::Matrix<T, N, N>& Eigenvectors() const;
        std::size_t Sweeps() const;

    private:
        static void ApplyPivotBlock(math::Matrix<T, N, N>& a, std::size_t p, std::size_t q, const math::GivensRotation<T>& rot);
        void Rotate(math::Matrix<T, N, N>& a, std::size_t p, std::size_t q);
        void SortAscending();

        math::Vector<T, N> eigenvalues{};
        math::Matrix<T, N, N> eigenvectors{};
        std::size_t sweeps{};
        bool solved{ false };

        static constexpr std::size_t maxSweeps{ 50 };
    };

    template<typename T, std::size_t N>
    void JacobiEigenSolver<T, N>::ApplyPivotBlock(
        math::Matrix<T, N, N>& a, std::size_t p, std::size_t q, const math::GivensRotation<T>& rot)
    {
        T c = rot.c;
        T s = rot.s;
        T app = a.at(p, p);
        T aqq = a.at(q, q);
        T apq = a.at(p, q);

        a.at(p, p) = c * c * app - T{ 2 } * s * c * apq + s * s * aqq;
        a.at(q, q) = s * s * app + T{ 2 } * s * c * apq + c * c * aqq;
        a.at(p, q) = T{};
        a.at(q, p) = T{};
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED void JacobiEigenSolver<T, N>::Rotate(math::Matrix<T, N, N>& a, std::size_t p, std::size_t q)
    {
        if (a.at(p, q) == T{})
            return;

        math::GivensRotation<T> rot = math::ComputeJacobiRotation(a.at(p, p), a.at(q, q), a.at(p, q));
        ApplyPivotBlock(a, p, q, rot);

        math::GivensRotation<T> transposed{ rot.c, -rot.s };

        for (std::size_t i = 0; i < N; ++i)
        {
            if (i != p && i != q)
            {
                math::ApplyGivens(transposed, a.at(i, p), a.at(i, q));
                a.at(p, i) = a.at(i, p);
                a.at(q, i) = a.at(i, q);
            }

            math::ApplyGivens(transposed, eigenvectors.at(i, p), eigenvectors.at(i, q));
        }
    }

    template<typename T, std::size_t N>
    void JacobiEigenSolver<T, N>::SortAscending()
    {
        for (std::size_t i = 0; i + 1 < N; ++i)
        {
            std::size_t minIndex = i;
            for (std::size_t j = i + 1; j < N; ++j)
                if (eigenvalues.at(j, 0) < eigenvalues.at(minIndex, 0))
                    minIndex = j;

            if (minIndex != i)
            {
                T tmp = eigenvalues.at(i, 0);
                eigenvalues.at(i, 0) = eigenvalues.at(minIndex, 0);
                eigenvalues.at(minIndex, 0) = tmp;

                for (std::size_t r = 0; r < N; ++r)
                {
                    T v = eigenvectors.at(r, i);
                    eigenvectors.at(r, i) = eigenvectors.at(r, minIndex);
                    eigenvectors.at(r, minIndex) = v;
                }
            }
        }
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED bool JacobiEigenSolver<T, N>::Solve(const math::Matrix<T, N, N>& a)
    {
        math::Matrix<T, N, N> work = a;
        eigenvectors = math::Matrix<T, N, N>::Identity();
        sweeps = 0;
        solved = false;

        T scale = math::OffDiagonalFrobeniusNorm(a);
        for (std::size_t i = 0; i < N; ++i)
            scale += std::abs(a.at(i, i));

        T threshold = ((scale > T{}) ? scale : T{ 1 }) * T{ 1e-7f };

        while (sweeps < maxSweeps)
        {
            if (math::OffDiagonalFrobeniusNorm(work) <= threshold)
                solved = true;

            if (solved)
                break;

            for (std::size_t p = 0; p < N; ++p)
                for (std::size_t q = p + 1; q < N; ++q)
                    Rotate(work, p, q);

            ++sweeps;
        }

        for (std::size_t i = 0; i < N; ++i)
            eigenvalues.at(i, 0) = work.at(i, i);

        SortAscending();
        return solved;
    }

    template<typename T, std::size_t N>
    const math::Vector<T, N>& JacobiEigenSolver<T, N>::Eigenvalues() const
    {
        return eigenvalues;
    }

    template<typename T, std::size_t N>
    const math::Matrix<T, N, N>& JacobiEigenSolver<T, N>::Eigenvectors() const
    {
        return eigenvectors;
    }

    template<typename T, std::size_t N>
    std::size_t JacobiEigenSolver<T, N>::Sweeps() const
    {
        return sweeps;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class JacobiEigenSolver<float, 2>;
    extern template class JacobiEigenSolver<float, 3>;
    extern template class JacobiEigenSolver<float, 4>;
#endif
}

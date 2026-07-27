#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace control_analysis
{
    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    class ControllabilityObservability
    {
        static_assert(std::is_floating_point_v<T>,
            "ControllabilityObservability supports floating-point types");

    public:
        using LTI = math::LinearTimeInvariant<T, n, m, p>;
        using CtrbMatrix = math::Matrix<T, n, n * m>;
        using ObsvMatrix = math::Matrix<T, n * p, n>;
        using SquareMatrix = math::Matrix<T, n, n>;

        OPTIMIZE_FOR_SPEED static CtrbMatrix ControllabilityMatrix(const LTI& plant);
        OPTIMIZE_FOR_SPEED static ObsvMatrix ObservabilityMatrix(const LTI& plant);

        template<std::size_t Rows, std::size_t Cols>
        static std::size_t Rank(const math::Matrix<T, Rows, Cols>& M, T tol);

        static bool IsControllable(const LTI& plant, T tol = T(1e-6));
        static bool IsObservable(const LTI& plant, T tol = T(1e-6));
        OPTIMIZE_FOR_SPEED static SquareMatrix ControllabilityGramian(const LTI& plant);
        OPTIMIZE_FOR_SPEED static SquareMatrix ObservabilityGramian(const LTI& plant);

    private:
        template<std::size_t Rows, std::size_t Cols>
        static T MaxAbsValue(const math::Matrix<T, Rows, Cols>& M);

        template<std::size_t Rows, std::size_t Cols>
        static void SwapRows(math::Matrix<T, Rows, Cols>& M, std::size_t r1, std::size_t r2);

        template<std::size_t Rows, std::size_t Cols>
        static void EliminateBelow(
            math::Matrix<T, Rows, Cols>& M, std::size_t pivotRow, std::size_t col, T pivotVal);

        static T MaxAbsDiff(const SquareMatrix& a, const SquareMatrix& b);
        static SquareMatrix SolveDiscreteLyapunov(const SquareMatrix& A, const SquareMatrix& Q);
    };

    ////    Implementation    ////

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    OPTIMIZE_FOR_SPEED typename ControllabilityObservability<T, n, m, p>::CtrbMatrix
    ControllabilityObservability<T, n, m, p>::ControllabilityMatrix(const LTI& plant)
    {
        CtrbMatrix result{};
        math::Matrix<T, n, m> Ak{ plant.B };

        for (std::size_t k = 0; k < n; ++k)
        {
            result.SetBlock(Ak, 0, k * m);
            Ak = plant.A * Ak;
        }

        return result;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    OPTIMIZE_FOR_SPEED typename ControllabilityObservability<T, n, m, p>::ObsvMatrix
    ControllabilityObservability<T, n, m, p>::ObservabilityMatrix(const LTI& plant)
    {
        ObsvMatrix result{};
        math::Matrix<T, p, n> CAk{ plant.C };

        for (std::size_t k = 0; k < n; ++k)
        {
            result.SetBlock(CAk, k * p, 0);
            CAk = CAk * plant.A;
        }

        return result;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    template<std::size_t Rows, std::size_t Cols>
    T ControllabilityObservability<T, n, m, p>::MaxAbsValue(const math::Matrix<T, Rows, Cols>& M)
    {
        T maxVal{ T(0) };
        for (std::size_t r = 0; r < Rows; ++r)
            for (std::size_t c = 0; c < Cols; ++c)
            {
                T v = M.at(r, c);
                if (v < T(0))
                    v = -v;
                if (v > maxVal)
                    maxVal = v;
            }
        return maxVal;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    template<std::size_t Rows, std::size_t Cols>
    void ControllabilityObservability<T, n, m, p>::SwapRows(
        math::Matrix<T, Rows, Cols>& M, std::size_t r1, std::size_t r2)
    {
        for (std::size_t c = 0; c < Cols; ++c)
        {
            T tmp = M.at(r1, c);
            M.at(r1, c) = M.at(r2, c);
            M.at(r2, c) = tmp;
        }
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    template<std::size_t Rows, std::size_t Cols>
    void ControllabilityObservability<T, n, m, p>::EliminateBelow(
        math::Matrix<T, Rows, Cols>& M, std::size_t pivotRow, std::size_t col, T pivotVal)
    {
        for (std::size_t r = pivotRow + 1; r < Rows; ++r)
        {
            T factor = M.at(r, col) / pivotVal;
            for (std::size_t c = col; c < Cols; ++c)
                M.at(r, c) -= factor * M.at(pivotRow, c);
        }
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    template<std::size_t Rows, std::size_t Cols>
    std::size_t ControllabilityObservability<T, n, m, p>::Rank(
        const math::Matrix<T, Rows, Cols>& M, T tol)
    {
        math::Matrix<T, Rows, Cols> copy{ M };
        T threshold{ tol * MaxAbsValue(copy) };
        std::size_t rank{ 0 };
        std::size_t pivotRow{ 0 };

        for (std::size_t col = 0; col < Cols && pivotRow < Rows; ++col)
        {
            std::size_t maxRow{ pivotRow };
            T maxVal{ T(0) };
            for (std::size_t r = pivotRow; r < Rows; ++r)
            {
                T v = copy.at(r, col);
                if (v < T(0))
                    v = -v;
                if (v > maxVal)
                {
                    maxVal = v;
                    maxRow = r;
                }
            }

            if (maxVal <= threshold)
                continue;

            SwapRows(copy, pivotRow, maxRow);
            EliminateBelow(copy, pivotRow, col, copy.at(pivotRow, col));
            ++rank;
            ++pivotRow;
        }

        return rank;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    bool ControllabilityObservability<T, n, m, p>::IsControllable(const LTI& plant, T tol)
    {
        return Rank(ControllabilityMatrix(plant), tol) == n;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    bool ControllabilityObservability<T, n, m, p>::IsObservable(const LTI& plant, T tol)
    {
        return Rank(ObservabilityMatrix(plant), tol) == n;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    T ControllabilityObservability<T, n, m, p>::MaxAbsDiff(
        const SquareMatrix& a, const SquareMatrix& b)
    {
        T maxDiff{ T(0) };
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t c = 0; c < n; ++c)
            {
                T d = a.at(r, c) - b.at(r, c);
                if (d < T(0))
                    d = -d;
                if (d > maxDiff)
                    maxDiff = d;
            }
        return maxDiff;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    typename ControllabilityObservability<T, n, m, p>::SquareMatrix
    ControllabilityObservability<T, n, m, p>::SolveDiscreteLyapunov(
        const SquareMatrix& A, const SquareMatrix& Q)
    {
        constexpr std::size_t maxIter{ 200 };
        constexpr T convTol{ T(1e-9) };
        SquareMatrix X{ Q };
        SquareMatrix At{ A.Transpose() };

        for (std::size_t iter = 0; iter < maxIter; ++iter)
        {
            SquareMatrix Xnext{ A * X * At + Q };
            if (MaxAbsDiff(Xnext, X) < convTol)
                return Xnext;
            X = Xnext;
        }

        return SquareMatrix{};
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    OPTIMIZE_FOR_SPEED typename ControllabilityObservability<T, n, m, p>::SquareMatrix
    ControllabilityObservability<T, n, m, p>::ControllabilityGramian(const LTI& plant)
    {
        return SolveDiscreteLyapunov(plant.A, plant.B * plant.B.Transpose());
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    OPTIMIZE_FOR_SPEED typename ControllabilityObservability<T, n, m, p>::SquareMatrix
    ControllabilityObservability<T, n, m, p>::ObservabilityGramian(const LTI& plant)
    {
        return SolveDiscreteLyapunov(plant.A.Transpose(), plant.C.Transpose() * plant.C);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ControllabilityObservability<float, 2, 1, 1>;
#endif
}

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include <cmath>
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
        static std::size_t Rank(const CtrbMatrix& M, T tol);
        static std::size_t RankObsv(const ObsvMatrix& M, T tol);
        static bool IsControllable(const LTI& plant, T tol = T(1e-6));
        static bool IsObservable(const LTI& plant, T tol = T(1e-6));
        OPTIMIZE_FOR_SPEED static SquareMatrix ControllabilityGramian(const LTI& plant);
        OPTIMIZE_FOR_SPEED static SquareMatrix ObservabilityGramian(const LTI& plant);

    private:
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
    std::size_t ControllabilityObservability<T, n, m, p>::Rank(const CtrbMatrix& M, T tol)
    {
        constexpr std::size_t rows = n;
        constexpr std::size_t cols = n * m;
        math::Matrix<T, rows, cols> copy{ M };

        T largestPivot{ T(0) };
        for (std::size_t r = 0; r < rows; ++r)
            for (std::size_t c = 0; c < cols; ++c)
            {
                T v = copy.at(r, c);
                if (v < T(0))
                    v = -v;
                if (v > largestPivot)
                    largestPivot = v;
            }

        T threshold{ tol * largestPivot };
        std::size_t rank{ 0 };
        std::size_t pivotRow{ 0 };

        for (std::size_t col = 0; col < cols && pivotRow < rows; ++col)
        {
            std::size_t maxRow{ pivotRow };
            T maxVal{ T(0) };
            for (std::size_t r = pivotRow; r < rows; ++r)
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

            for (std::size_t c = 0; c < cols; ++c)
            {
                T tmp = copy.at(pivotRow, c);
                copy.at(pivotRow, c) = copy.at(maxRow, c);
                copy.at(maxRow, c) = tmp;
            }

            T pivotVal = copy.at(pivotRow, col);
            for (std::size_t r = pivotRow + 1; r < rows; ++r)
            {
                T factor = copy.at(r, col) / pivotVal;
                for (std::size_t c = col; c < cols; ++c)
                    copy.at(r, c) -= factor * copy.at(pivotRow, c);
            }

            ++rank;
            ++pivotRow;
        }

        return rank;
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    std::size_t ControllabilityObservability<T, n, m, p>::RankObsv(const ObsvMatrix& M, T tol)
    {
        constexpr std::size_t rows = n * p;
        constexpr std::size_t cols = n;
        math::Matrix<T, rows, cols> copy{ M };

        T largestPivot{ T(0) };
        for (std::size_t r = 0; r < rows; ++r)
            for (std::size_t c = 0; c < cols; ++c)
            {
                T v = copy.at(r, c);
                if (v < T(0))
                    v = -v;
                if (v > largestPivot)
                    largestPivot = v;
            }

        T threshold{ tol * largestPivot };
        std::size_t rank{ 0 };
        std::size_t pivotRow{ 0 };

        for (std::size_t col = 0; col < cols && pivotRow < rows; ++col)
        {
            std::size_t maxRow{ pivotRow };
            T maxVal{ T(0) };
            for (std::size_t r = pivotRow; r < rows; ++r)
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

            for (std::size_t c = 0; c < cols; ++c)
            {
                T tmp = copy.at(pivotRow, c);
                copy.at(pivotRow, c) = copy.at(maxRow, c);
                copy.at(maxRow, c) = tmp;
            }

            T pivotVal = copy.at(pivotRow, col);
            for (std::size_t r = pivotRow + 1; r < rows; ++r)
            {
                T factor = copy.at(r, col) / pivotVal;
                for (std::size_t c = col; c < cols; ++c)
                    copy.at(r, c) -= factor * copy.at(pivotRow, c);
            }

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
        return RankObsv(ObservabilityMatrix(plant), tol) == n;
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

            T maxDiff{ T(0) };
            for (std::size_t r = 0; r < n; ++r)
                for (std::size_t c = 0; c < n; ++c)
                {
                    T d = Xnext.at(r, c) - X.at(r, c);
                    if (d < T(0))
                        d = -d;
                    if (d > maxDiff)
                        maxDiff = d;
                }

            X = Xnext;

            if (maxDiff < convTol)
                return X;
        }

        return SquareMatrix{};
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    OPTIMIZE_FOR_SPEED typename ControllabilityObservability<T, n, m, p>::SquareMatrix
    ControllabilityObservability<T, n, m, p>::ControllabilityGramian(const LTI& plant)
    {
        SquareMatrix Q{ plant.B * plant.B.Transpose() };
        return SolveDiscreteLyapunov(plant.A, Q);
    }

    template<typename T, std::size_t n, std::size_t m, std::size_t p>
    OPTIMIZE_FOR_SPEED typename ControllabilityObservability<T, n, m, p>::SquareMatrix
    ControllabilityObservability<T, n, m, p>::ObservabilityGramian(const LTI& plant)
    {
        SquareMatrix Q{ plant.C.Transpose() * plant.C };
        SquareMatrix At{ plant.A.Transpose() };
        return SolveDiscreteLyapunov(At, Q);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ControllabilityObservability<float, 2, 1, 1>;
#endif
}

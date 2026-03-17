#pragma once

#include <cmath>
#include <type_traits>

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/Solver.hpp"

namespace solvers
{
    template<typename T, std::size_t N>
    class GaussianElimination
        : public Solver<T, N>
    {
    public:
        using typename Solver<T, N>::SolutionVector;
        using typename Solver<T, N>::InputMatrix;
        using typename Solver<T, N>::InputVector;

        GaussianElimination() = default;
        SolutionVector Solve(const InputMatrix& a, const InputVector& b) override;
    };

    template<typename T, std::size_t N, std::size_t Cols>
    math::Matrix<T, N, Cols> SolveSystem(const math::Matrix<T, N, N>& a, const math::Matrix<T, N, Cols>& b);

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED
        typename GaussianElimination<T, N>::SolutionVector
        GaussianElimination<T, N>::Solve(const InputMatrix& a, const InputVector& b)
    {
        InputMatrix augA = a;
        SolutionVector augB = b;

        for (std::size_t col = 0; col < N; ++col)
        {
            std::size_t pivotRow = col;
            T pivotVal = augA.at(col, col);

            if constexpr (std::is_same_v<T, float>)
            {
                float maxVal = std::abs(pivotVal);

                for (std::size_t row = col + 1; row < N; ++row)
                {
                    float absVal = std::abs(augA.at(row, col));
                    if (absVal > maxVal)
                    {
                        maxVal = absVal;
                        pivotRow = row;
                    }
                }
            }
            else
            {
                float maxVal = std::abs(pivotVal.ToFloat());

                for (std::size_t row = col + 1; row < N; ++row)
                {
                    float absVal = std::abs(augA.at(row, col).ToFloat());
                    if (absVal > maxVal)
                    {
                        maxVal = absVal;
                        pivotRow = row;
                    }
                }
            }

            if (pivotRow != col)
            {
                for (std::size_t j = 0; j < N; ++j)
                {
                    T tmp = augA.at(col, j);
                    augA.at(col, j) = augA.at(pivotRow, j);
                    augA.at(pivotRow, j) = tmp;
                }

                T tmp = augB.at(col, 0);
                augB.at(col, 0) = augB.at(pivotRow, 0);
                augB.at(pivotRow, 0) = tmp;
            }

            T pivot = augA.at(col, col);

            for (std::size_t row = col + 1; row < N; ++row)
            {
                T factor = augA.at(row, col) / pivot;

                for (std::size_t j = col; j < N; ++j)
                    augA.at(row, j) = augA.at(row, j) - factor * augA.at(col, j);

                augB.at(row, 0) = augB.at(row, 0) - factor * augB.at(col, 0);
            }
        }

        SolutionVector x;

        for (std::size_t i = N; i > 0; --i)
        {
            std::size_t row = i - 1;
            T sum = augB.at(row, 0);

            for (std::size_t j = row + 1; j < N; ++j)
                sum = sum - augA.at(row, j) * x.at(j, 0);

            x.at(row, 0) = sum / augA.at(row, row);
        }

        return x;
    }

    template<typename T, std::size_t N, std::size_t Cols>
    OPTIMIZE_FOR_SPEED
        math::Matrix<T, N, Cols>
        SolveSystem(const math::Matrix<T, N, N>& a, const math::Matrix<T, N, Cols>& b)
    {
        GaussianElimination<T, N> solver;
        math::Matrix<T, N, Cols> result;

        for (std::size_t col = 0; col < Cols; ++col)
        {
            math::Vector<T, N> bCol;
            for (std::size_t row = 0; row < N; ++row)
                bCol.at(row, 0) = b.at(row, col);

            auto xCol = solver.Solve(a, bCol);

            for (std::size_t row = 0; row < N; ++row)
                result.at(row, col) = xCol.at(row, 0);
        }

        return result;
    }

    template<typename T, std::size_t N>
    [[nodiscard]] constexpr GaussianElimination<T, N> MakeGaussianElimination()
    {
        return GaussianElimination<T, N>();
    }
}

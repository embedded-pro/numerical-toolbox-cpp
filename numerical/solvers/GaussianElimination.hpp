#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/TriangularSolve.hpp"
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

    private:
        void EliminateBelow(InputMatrix& matrix, SolutionVector& vector, std::size_t col) const;
    };

    template<typename T, std::size_t N, std::size_t Cols>
    math::Matrix<T, N, Cols> SolveSystem(const math::Matrix<T, N, N>& a, const math::Matrix<T, N, Cols>& b);

    template<typename T, std::size_t N>
    void GaussianElimination<T, N>::EliminateBelow(InputMatrix& matrix, SolutionVector& vector, std::size_t col) const
    {
        T pivot = matrix.at(col, col);
        really_assert(math::Abs(math::ToFloat(pivot)) > 0.0f);

        for (std::size_t row = col + 1; row < N; ++row)
        {
            T factor = matrix.at(row, col) / pivot;

            for (std::size_t j = col; j < N; ++j)
                matrix.at(row, j) = matrix.at(row, j) - factor * matrix.at(col, j);

            vector.at(row, 0) = vector.at(row, 0) - factor * vector.at(col, 0);
        }
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED
        typename GaussianElimination<T, N>::SolutionVector
        GaussianElimination<T, N>::Solve(const InputMatrix& a, const InputVector& b)
    {
        InputMatrix augA = a;
        SolutionVector augB = b;

        for (std::size_t col = 0; col < N; ++col)
        {
            std::size_t pivotRow = math::FindPartialPivotRow(augA, col);

            if (pivotRow != col)
            {
                math::SwapRows(augA, col, pivotRow);
                math::SwapRows(augB, col, pivotRow);
            }

            EliminateBelow(augA, augB, col);
        }

        return math::SolveUpperTriangular(augA, augB);
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

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class GaussianElimination<float, 1>;
    extern template class GaussianElimination<float, 2>;
    extern template class GaussianElimination<float, 3>;

    extern template class GaussianElimination<math::Q15, 3>;

    extern template class GaussianElimination<math::Q31, 3>;
#endif
}

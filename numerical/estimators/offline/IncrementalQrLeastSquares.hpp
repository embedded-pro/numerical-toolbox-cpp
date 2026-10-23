#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include <cstddef>
#include <optional>
#include <type_traits>

namespace estimators
{
    template<typename T, std::size_t Parameters>
    struct LeastSquaresResult
    {
        static_assert(std::is_floating_point_v<T>, "LeastSquaresResult supports floating-point types only");

        math::Vector<T, Parameters> parameters{};
        std::size_t rank{ 0 };
        math::SquareMatrix<T, Parameters> rowSpace{};
        T residualRms{};
    };

    template<typename T, std::size_t Parameters>
    class IncrementalQrLeastSquares
    {
        static_assert(std::is_floating_point_v<T>, "IncrementalQrLeastSquares supports floating-point types only");

    public:
        using Regressor = math::Vector<T, Parameters>;
        using Result = LeastSquaresResult<T, Parameters>;

        OPTIMIZE_FOR_SPEED void AddRow(const Regressor& row, T rhs);
        template<std::size_t BlockRows>
        void AddRows(const math::Matrix<T, BlockRows, Parameters>& block, const math::Vector<T, BlockRows>& rhs);
        std::optional<Result> Solve(T relativeTolerance) const;
        void Reset();
        std::size_t Rows() const;

    private:
        math::SquareMatrix<T, Parameters> r{};
        Regressor d{};
        T residualSquares{};
        std::size_t rows{ 0 };
    };

    template<typename T, std::size_t Parameters>
    OPTIMIZE_FOR_SPEED void IncrementalQrLeastSquares<T, Parameters>::AddRow(const Regressor& row, T rhs)
    {
        Regressor x{ row };
        for (std::size_t j = 0; j < Parameters; ++j)
        {
            if (x[j] == T{})
                continue;

            const auto rotation{ math::ComputeGivens(r.at(j, j), x[j]) };
            for (std::size_t c = j; c < Parameters; ++c)
                math::ApplyGivens(rotation, r.at(j, c), x[c]);
            math::ApplyGivens(rotation, d[j], rhs);
        }

        residualSquares += rhs * rhs;
        ++rows;
    }

    template<typename T, std::size_t Parameters>
    template<std::size_t BlockRows>
    void IncrementalQrLeastSquares<T, Parameters>::AddRows(const math::Matrix<T, BlockRows, Parameters>& block, const math::Vector<T, BlockRows>& rhs)
    {
        for (std::size_t i = 0; i < BlockRows; ++i)
            AddRow(block.template GetBlock<1, Parameters>(i, 0).Transpose(), rhs[i]);
    }

    template<typename T, std::size_t Parameters>
    auto IncrementalQrLeastSquares<T, Parameters>::Solve(T relativeTolerance) const -> std::optional<Result>
    {
        solvers::SingularValueDecomposition<T, Parameters, Parameters> svd;
        if (rows == 0 || !svd.Decompose(r))
            return std::nullopt;

        const auto& sigma{ svd.SingularValues() };
        const auto& u{ svd.U() };
        const auto& v{ svd.V() };
        const T threshold{ relativeTolerance * sigma[0] };

        Result result{};
        T unexplained{ residualSquares };
        for (std::size_t i = 0; i < Parameters; ++i)
        {
            T projection{};
            for (std::size_t k = 0; k < Parameters; ++k)
                projection += u.at(k, i) * d[k];

            if (sigma[i] > threshold)
            {
                ++result.rank;
                for (std::size_t k = 0; k < Parameters; ++k)
                    result.parameters[k] += v.at(k, i) * projection / sigma[i];
            }
            else
                unexplained += projection * projection;
        }

        result.rowSpace = v.Transpose();
        result.residualRms = math::Sqrt(unexplained / static_cast<T>(rows));
        return result;
    }

    template<typename T, std::size_t Parameters>
    void IncrementalQrLeastSquares<T, Parameters>::Reset()
    {
        r = math::SquareMatrix<T, Parameters>{};
        d = Regressor{};
        residualSquares = T{};
        rows = 0;
    }

    template<typename T, std::size_t Parameters>
    std::size_t IncrementalQrLeastSquares<T, Parameters>::Rows() const
    {
        return rows;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class IncrementalQrLeastSquares<float, 2>;
    extern template class IncrementalQrLeastSquares<float, 3>;
#endif
}

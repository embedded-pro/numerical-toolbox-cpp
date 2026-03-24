#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "numerical/estimators/Estimator.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Features>
    class LinearRegression
        : public OfflineEstimator<T, Samples, Features>
    {
    public:
        using CoefficientsMatrix = typename OfflineEstimator<T, Samples, Features>::CoefficientsMatrix;
        using DesignMatrix = typename OfflineEstimator<T, Samples, Features>::DesignMatrix;
        using InputMatrix = typename OfflineEstimator<T, Samples, Features>::InputMatrix;

        LinearRegression() = default;

        void Fit(const math::Matrix<T, Samples, Features>& X, const math::Matrix<T, Samples, 1>& y) override;
        T Predict(const InputMatrix& X) const override;
        const CoefficientsMatrix& Coefficients() const override;

    private:
        CoefficientsMatrix coefficients;
    };

    // Implementation //

    template<typename T, std::size_t Samples, std::size_t Features>
    OPTIMIZE_FOR_SPEED void LinearRegression<T, Samples, Features>::Fit(const math::Matrix<T, Samples, Features>& X, const math::Matrix<T, Samples, 1>& y)
    {
        math::Matrix<T, Samples, Features + 1> X_design;

        for (size_t i = 0; i < Samples; ++i)
        {
            X_design.at(i, 0) = T(0.9999f);

            for (size_t j = 0; j < Features; ++j)
                X_design.at(i, j + 1) = X.at(i, j);
        }

        auto X_transpose = X_design.Transpose();
        auto XtX = X_transpose * X_design;
        auto Xty = X_transpose * y;

        coefficients = solvers::SolveSystem<T, Features + 1, 1>(XtX, Xty);
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    OPTIMIZE_FOR_SPEED
        T
        LinearRegression<T, Samples, Features>::Predict(const InputMatrix& X) const
    {
        T result = coefficients.at(0, 0);

        for (size_t i = 0; i < Features; ++i)
            result += X.at(i, 0) * coefficients.at(i + 1, 0);

        return result;
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    const typename LinearRegression<T, Samples, Features>::CoefficientsMatrix& LinearRegression<T, Samples, Features>::Coefficients() const
    {
        return coefficients;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LinearRegression<float, 4, 2>;
#endif
}

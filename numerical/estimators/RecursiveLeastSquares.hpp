#pragma once

#include "numerical/estimators/Estimator.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace estimators
{
    enum class State
    {
        unstable,
        converged,
    };

    template<typename T, std::size_t Features>
    class RecursiveLeastSquares
        : public OnlineEstimator<T, Features>
    {
    public:
        using CoefficientsMatrix = typename OnlineEstimator<T, Features>::CoefficientsMatrix;
        using DesignMatrix = typename OnlineEstimator<T, Features>::DesignMatrix;
        using InputMatrix = typename OnlineEstimator<T, Features>::InputMatrix;
        using EstimationMetrics = typename OnlineEstimator<T, Features>::EstimationMetrics;

        explicit RecursiveLeastSquares(T forgettingFactor);
        RecursiveLeastSquares(T initialCovariance, T forgettingFactor);

        EstimationMetrics Update(const InputMatrix& X, const math::Matrix<T, 1, 1>& y) override;
        const CoefficientsMatrix& Coefficients() const override;

        template<typename... Args>
        static InputMatrix& MakeRegressor(InputMatrix& regressor, Args&&... args);

        static State EvaluateConvergence(const EstimationMetrics& metrics, T innovationThreshold, T uncertaintyThreshold) noexcept;

    private:
        CoefficientsMatrix theta;
        DesignMatrix covariance;
        T lambda;
        T lambdaInverse;
        EstimationMetrics metrics;
    };

    // Implementation

    template<typename T, std::size_t Features>
    RecursiveLeastSquares<T, Features>::RecursiveLeastSquares(T forgettingFactor)
        : theta{}
        , covariance(DesignMatrix::Identity())
        , lambda(forgettingFactor)
        , lambdaInverse(T(1) / lambda)
    {
    }

    template<typename T, std::size_t Features>
    RecursiveLeastSquares<T, Features>::RecursiveLeastSquares(T initialCovariance, T forgettingFactor)
        : theta{}
        , covariance(DesignMatrix::Identity() * initialCovariance)
        , lambda(forgettingFactor)
        , lambdaInverse(T(1) / lambda)
    {
    }

    template<typename T, std::size_t Features>
    OPTIMIZE_FOR_SPEED
        typename RecursiveLeastSquares<T, Features>::EstimationMetrics
        RecursiveLeastSquares<T, Features>::Update(const InputMatrix& x, const math::Matrix<T, 1, 1>& y)
    {
        auto Px = covariance * x;
        auto denominatorInv = T(1) / (lambda + (x.Transpose() * Px).at(0, 0));
        auto error = y.at(0, 0) - (x.Transpose() * theta).at(0, 0);
        theta += Px * (error * denominatorInv);
        covariance = (covariance - Px * Px.Transpose() * denominatorInv) * lambdaInverse;

        metrics.innovation = error;
        metrics.residual = y.at(0, 0) - (x.Transpose() * theta).at(0, 0);
        metrics.uncertainty = covariance.Trace();

        return metrics;
    }

    template<typename T, std::size_t Features>
    const typename RecursiveLeastSquares<T, Features>::CoefficientsMatrix& RecursiveLeastSquares<T, Features>::Coefficients() const
    {
        return theta;
    }

    template<typename T, std::size_t Features>
    template<typename... Args>
    typename RecursiveLeastSquares<T, Features>::InputMatrix& RecursiveLeastSquares<T, Features>::MakeRegressor(InputMatrix& regressor, Args&&... args)
    {
        static_assert(sizeof...(Args) == Features - 1, "Number of arguments must match Features - 1");

        regressor.at(0, 0) = T(1);
        std::size_t i = 1;
        ((regressor.at(i++, 0) = std::forward<Args>(args)), ...);
        return regressor;
    }

    template<typename T, std::size_t Features>
    OPTIMIZE_FOR_SPEED
        State
        RecursiveLeastSquares<T, Features>::EvaluateConvergence(const EstimationMetrics& metrics, T innovationThreshold, T uncertaintyThreshold) noexcept
    {
        if (std::abs(metrics.innovation) < innovationThreshold && metrics.uncertainty < uncertaintyThreshold) [[likely]]
            return State::converged;
        else
            return State::unstable;
    }
}

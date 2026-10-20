#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/estimators/Estimator.hpp"
#include "numerical/estimators/online/RecursiveLeastSquares.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <type_traits>

namespace estimators
{
    template<typename T, std::size_t Features>
    class RecursiveInstrumentalVariables
    {
        static_assert(std::is_floating_point_v<T>, "RecursiveInstrumentalVariables supports floating-point types");

    public:
        using CoefficientsMatrix = typename OnlineEstimator<T, Features>::CoefficientsMatrix;
        using DesignMatrix = typename OnlineEstimator<T, Features>::DesignMatrix;
        using InputMatrix = typename OnlineEstimator<T, Features>::InputMatrix;
        using EstimationMetrics = typename OnlineEstimator<T, Features>::EstimationMetrics;

        RecursiveInstrumentalVariables(T initialCovariance, T forgettingFactor);

        EstimationMetrics Update(const InputMatrix& regressor, const InputMatrix& instrument, const math::Matrix<T, 1, 1>& y);
        const CoefficientsMatrix& Coefficients() const;
        void SetCoefficients(const CoefficientsMatrix& initial);

    private:
        CoefficientsMatrix theta{};
        DesignMatrix covariance;
        T lambda;
        T lambdaInverse;
        EstimationMetrics metrics{};
    };

    template<typename T, std::size_t Features>
    RecursiveInstrumentalVariables<T, Features>::RecursiveInstrumentalVariables(T initialCovariance, T forgettingFactor)
        : covariance{ DesignMatrix::Identity() * initialCovariance }
        , lambda{ forgettingFactor }
        , lambdaInverse{ T(1) / forgettingFactor }
    {
        really_assert((RecursiveLeastSquares<T, Features>::IsForgettingFactorValid(forgettingFactor)));
        really_assert(math::IsFinite(initialCovariance) && initialCovariance > T(0));
    }

    template<typename T, std::size_t Features>
    OPTIMIZE_FOR_SPEED
        typename RecursiveInstrumentalVariables<T, Features>::EstimationMetrics
        RecursiveInstrumentalVariables<T, Features>::Update(const InputMatrix& regressor, const InputMatrix& instrument, const math::Matrix<T, 1, 1>& y)
    {
        const auto gain = covariance * instrument;
        const auto denominatorInverse = T(1) / (lambda + (regressor.Transpose() * gain).at(0, 0));
        const auto error = y.at(0, 0) - (regressor.Transpose() * theta).at(0, 0);
        theta += gain * (error * denominatorInverse);
        covariance = (covariance - gain * (regressor.Transpose() * covariance) * denominatorInverse) * lambdaInverse;

        metrics.innovation = error;
        metrics.residual = y.at(0, 0) - (regressor.Transpose() * theta).at(0, 0);
        metrics.uncertainty = covariance.Trace();

        return metrics;
    }

    template<typename T, std::size_t Features>
    const typename RecursiveInstrumentalVariables<T, Features>::CoefficientsMatrix& RecursiveInstrumentalVariables<T, Features>::Coefficients() const
    {
        return theta;
    }

    template<typename T, std::size_t Features>
    void RecursiveInstrumentalVariables<T, Features>::SetCoefficients(const CoefficientsMatrix& initial)
    {
        theta = initial;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class RecursiveInstrumentalVariables<float, 2>;
#endif
}

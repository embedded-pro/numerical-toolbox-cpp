#pragma once

#include "numerical/estimators/Estimator.hpp"

namespace estimators
{
    template<typename T, std::size_t Features>
    class RecursiveLeastSquares
        : public OnlineEstimator<T, Features>
    {
    public:
        using CoefficientsMatrix = typename OnlineEstimator<T, Features>::CoefficientsMatrix;
        using DesignMatrix = typename OnlineEstimator<T, Features>::DesignMatrix;
        using InputMatrix = typename OnlineEstimator<T, Features>::InputMatrix;

        RecursiveLeastSquares(std::optional<T> initialCovariance, T forgettingFactor);

        void Update(const InputMatrix& X, const math::Matrix<T, 1, 1>& y) override;
        const CoefficientsMatrix& Coefficients() const override;

        template<typename... Args>
        static InputMatrix& MakeRegressor(InputMatrix& regressor, Args&&... args);

    private:
        CoefficientsMatrix theta;
        DesignMatrix covariance;
        T lambda;
        T lambdaInverse;
    };

    // Implementation

    template<typename T, std::size_t Features>
    RecursiveLeastSquares<T, Features>::RecursiveLeastSquares(std::optional<T> initialCovariance, T forgettingFactor)
        : theta{}
        , covariance(initialCovariance.has_value()
                         ? DesignMatrix::Identity() * initialCovariance.value()
                         : DesignMatrix::Identity())
        , lambda(forgettingFactor)
        , lambdaInverse(T(1) / lambda)
    {
    }

    template<typename T, std::size_t Features>
    void RecursiveLeastSquares<T, Features>::Update(const InputMatrix& x, const math::Matrix<T, 1, 1>& y)
    {
        auto Px = covariance * x;
        auto denominatorInv = T(1) / (lambda + (x.Transpose() * Px).at(0, 0));
        auto error = y.at(0, 0) - (x.Transpose() * theta).at(0, 0);
        theta += Px * (error * denominatorInv);
        covariance = (covariance - Px * Px.Transpose() * denominatorInv) * lambdaInverse;
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
}

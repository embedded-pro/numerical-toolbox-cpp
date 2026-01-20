#pragma once

#include "numerical/math/Matrix.hpp"

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Features>
    class OfflineEstimator
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "OfflineEstimator only supports float or QNumber types");

    public:
        using CoefficientsMatrix = math::Matrix<T, Features + 1, 1>;
        using DesignMatrix = math::Matrix<T, Features + 1, Features + 1>;
        using InputMatrix = math::Matrix<T, Features, 1>;

        virtual void Fit(const math::Matrix<T, Samples, Features>& X, const math::Matrix<T, Samples, 1>& y) = 0;
        virtual T Predict(const InputMatrix& X) const = 0;
        virtual const CoefficientsMatrix& Coefficients() const = 0;
    };

    template<typename T, std::size_t Features>
    class OnlineEstimator
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "OnlineEstimator only supports float or QNumber types");

    public:
        using CoefficientsMatrix = math::Matrix<T, Features, 1>;
        using DesignMatrix = math::Matrix<T, Features, Features>;
        using InputMatrix = math::Matrix<T, Features, 1>;

        struct EstimationMetrics
        {
            T innovation;  // The error BEFORE the update (how surprised we were)
            T residual;    // The error AFTER the update (how well we matched it)
            T uncertainty; // The 'Trace' or 'Norm' of P (how confident we are)
        };

        virtual EstimationMetrics Update(const InputMatrix& x, const math::Matrix<T, 1, 1>& y) = 0;
        virtual const CoefficientsMatrix& Coefficients() const = 0;
    };
}

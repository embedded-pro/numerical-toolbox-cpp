#ifndef ESTIMATORS_YULE_WALKER_HPP
#define ESTIMATORS_YULE_WALKER_HPP

#include "estimators/Estimator.hpp"

namespace estimators::solver
{
    template<typename T, std::size_t N>
    using ToeplitzMatrix = math::Matrix<T, N, N>;

    template<typename T, std::size_t N>
    ToeplitzMatrix<T, N> MakeToeplitzMatrix(const math::Matrix<T, N, 1>& autocorrelationVector)
    {
        ToeplitzMatrix<T, N> result;

        for (size_t i = 0; i < N; ++i)
            for (size_t j = 0; j < N; ++j)
                result.at(i, j) = autocorrelationVector.at(std::abs(static_cast<int>(i - j)), 0);

        return result;
    }

    template<typename T, std::size_t N>
    math::Matrix<T, N, 1> LevinsonDurbin(const math::Matrix<T, N, 1>& autocorrelationVector)
    {
        math::Matrix<T, N, 1> phi;
        T error = autocorrelationVector.at(0, 0);

        for (size_t k = 1; k < N; ++k)
        {
            T mu = autocorrelationVector.at(k, 0);
            for (size_t j = 1; j < k; ++j)
                mu -= phi.at(j, 0) * autocorrelationVector.at(k - j, 0);
            mu /= error;

            phi.at(k, 0) = mu;
            for (size_t j = 1; j < k; ++j)
            {
                T prev_phi_j = phi.at(j, 0);
                phi.at(j, 0) = prev_phi_j - mu * phi.at(k - j, 0);
            }

            error *= (T(1) - mu * mu);
        }

        return phi;
    }
}

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Order>
    class YuleWalker
        : public Estimator<T, Samples, Order>
    {
    public:
        using CoefficientsMatrix = typename Estimator<T, Samples, Order>::CoefficientsMatrix;
        using DesignMatrix = typename Estimator<T, Samples, Order>::DesignMatrix;
        using InputMatrix = typename Estimator<T, Samples, Order>::InputMatrix;
        using TimeSeriesVector = math::Matrix<T, Samples, 1>;
        using AutocorrVector = math::Matrix<T, Order + 1, 1>;

        YuleWalker() = default;

        void Fit(const math::Matrix<T, Samples, Order>& X, const math::Matrix<T, Samples, 1>& y) override;
        T Predict(const InputMatrix& X) const override;
        const CoefficientsMatrix& Coefficients() const override;

    private:
        CoefficientsMatrix coefficients;
        T mean{};

        // Helper functions
        T ComputeAutocorrelation(const TimeSeriesVector& timeSeries, size_t lag) const;
        TimeSeriesVector CenterTimeSeries(const TimeSeriesVector& timeSeries) const;
        AutocorrVector ComputeAutocorrelations(const TimeSeriesVector& centered) const;
    };

    // Implementation //

    template<typename T, std::size_t Samples, std::size_t Order>
    void YuleWalker<T, Samples, Order>::Fit(
        const math::Matrix<T, Samples, Order>& X,
        const math::Matrix<T, Samples, 1>& y)
    {
        // Compute and remove mean
        T sum{};
        for (size_t i = 0; i < Samples; ++i)
            sum += y.at(i, 0);
        mean = sum / T(Samples);

        // Center the time series and compute autocorrelations
        auto centered = CenterTimeSeries(y);
        auto autocorr = ComputeAutocorrelations(centered);

        // Solve Yule-Walker equations using Levinson-Durbin recursion
        coefficients = estimators::solver::LevinsonDurbin(autocorr);
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    T YuleWalker<T, Samples, Order>::Predict(const InputMatrix& X) const
    {
        T result = mean;
        for (size_t i = 0; i < Order; ++i)
            result += X.at(i, 0) * coefficients.at(i + 1, 0);
        return result;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    const typename YuleWalker<T, Samples, Order>::CoefficientsMatrix&
    YuleWalker<T, Samples, Order>::Coefficients() const
    {
        return coefficients;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    T YuleWalker<T, Samples, Order>::ComputeAutocorrelation(
        const TimeSeriesVector& timeSeries,
        size_t lag) const
    {
        T sum{};
        size_t n = Samples - lag;

        for (size_t i = 0; i < n; ++i)
            sum += timeSeries.at(i, 0) * timeSeries.at(i + lag, 0);

        return sum / T(n);
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    typename YuleWalker<T, Samples, Order>::TimeSeriesVector
    YuleWalker<T, Samples, Order>::CenterTimeSeries(
        const TimeSeriesVector& timeSeries) const
    {
        TimeSeriesVector centered;
        for (size_t i = 0; i < Samples; ++i)
            centered.at(i, 0) = timeSeries.at(i, 0) - mean;
        return centered;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    typename YuleWalker<T, Samples, Order>::AutocorrVector
    YuleWalker<T, Samples, Order>::ComputeAutocorrelations(
        const TimeSeriesVector& centered) const
    {
        AutocorrVector autocorr;
        for (size_t i = 0; i <= Order; ++i)
            autocorr.at(i, 0) = ComputeAutocorrelation(centered, i);
        return autocorr;
    }
}

#endif

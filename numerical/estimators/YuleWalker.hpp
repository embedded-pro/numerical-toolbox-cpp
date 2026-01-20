#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/Toeplitz.hpp"
#include "numerical/solvers/Solver.hpp"

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Order>
    class YuleWalker
    {
    public:
        using CoefficientsMatrix = typename math::Matrix<T, Order, 1>;
        using DesignMatrix = math::Matrix<T, Order, Order>;
        using InputMatrix = math::Matrix<T, Order, 1>;
        using TimeSeriesVector = math::Matrix<T, Samples, 1>;

        explicit YuleWalker(solvers::Solver<T, Order>& solver);

        void Fit(const math::Matrix<T, Samples, Order>& X, const math::Matrix<T, Samples, 1>& y);
        T Predict(const InputMatrix& X) const;
        const CoefficientsMatrix& Coefficients() const;

    private:
        CoefficientsMatrix coefficients;
        solvers::Solver<T, Order>& solver;
        T mean{};

        T ComputeMean(const TimeSeriesVector& timeSeries) const;
        T ComputeAutocovariance(const TimeSeriesVector& timeSeries, size_t lag) const;
    };

    // Implementation //

    template<typename T, std::size_t Samples, std::size_t Order>
    YuleWalker<T, Samples, Order>::YuleWalker(solvers::Solver<T, Order>& solver)
        : solver(solver)
    {}

    template<typename T, std::size_t Samples, std::size_t Order>
    void YuleWalker<T, Samples, Order>::Fit(const math::Matrix<T, Samples, Order>& X, const math::Matrix<T, Samples, 1>& y)
    {
        mean = ComputeMean(y);
        TimeSeriesVector centered_y;
        for (size_t i = 0; i < Samples; ++i)
            centered_y.at(i, 0) = y.at(i, 0) - mean;

        math::Vector<T, Order> gamma;
        for (size_t i = 0; i < Order; ++i)
            gamma.at(i, 0) = ComputeAutocovariance(centered_y, i);

        math::ToeplitzMatrix<T, Order> toeplitz(gamma);

        math::Vector<T, Order> rhs;
        for (size_t i = 0; i < Order; ++i)
            rhs.at(i, 0) = ComputeAutocovariance(centered_y, i + 1);

        coefficients = solver.Solve(toeplitz.ToFullMatrix(), rhs);
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    T YuleWalker<T, Samples, Order>::Predict(const InputMatrix& X) const
    {
        T result = mean;

        for (size_t i = 0; i < Order; ++i)
            result += X.at(i, 0) * coefficients.at(i, 0);

        return result;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    const typename YuleWalker<T, Samples, Order>::CoefficientsMatrix&
    YuleWalker<T, Samples, Order>::Coefficients() const
    {
        return coefficients;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    T YuleWalker<T, Samples, Order>::ComputeMean(const TimeSeriesVector& timeSeries) const
    {
        T sum{};

        for (size_t i = 0; i < Samples; ++i)
            sum += timeSeries.at(i, 0);

        return sum / T(Samples);
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    T YuleWalker<T, Samples, Order>::ComputeAutocovariance(const TimeSeriesVector& timeSeries, size_t lag) const
    {
        T covariance{};
        size_t n = 0;

        for (size_t i = lag; i < Samples; ++i)
        {
            covariance += timeSeries.at(i, 0) * timeSeries.at(i - lag, 0);
            n++;
        }

        return covariance / T(Samples);
    }
}

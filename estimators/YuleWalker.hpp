#ifndef ESTIMATORS_YULE_WALKER_HPP
#define ESTIMATORS_YULE_WALKER_HPP

#include "math/Matrix.hpp"
#include "math/Toeplitz.hpp"
#include "solvers/Solver.hpp"

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
        TimeSeriesVector CenterTimeSeries(const TimeSeriesVector& timeSeries) const;
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
        auto centered_y = CenterTimeSeries(y);

        math::Vector<T, Order> toeplitz_elements;

        for (size_t i = 0; i < Order; ++i)
        {
            T sum = T(0);

            for (size_t k = 0; k < Samples; ++k)
                sum += X.at(k, 0) * X.at(k, i);

            toeplitz_elements.at(i, 0) = sum / T(Samples);
        }

        math::ToeplitzMatrix<T, Order> toeplitz(toeplitz_elements);

        math::Vector<T, Order> rhs;
        for (size_t i = 0; i < Order; ++i)
        {
            T sum = T(0);

            for (size_t k = 0; k < Samples; ++k)
                sum += X.at(k, i) * centered_y.at(k, 0);

            rhs.at(i, 0) = sum / T(Samples);
        }

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
    typename YuleWalker<T, Samples, Order>::TimeSeriesVector
    YuleWalker<T, Samples, Order>::CenterTimeSeries(const TimeSeriesVector& timeSeries) const
    {
        TimeSeriesVector centered;

        for (size_t i = 0; i < Samples; ++i)
            centered.at(i, 0) = timeSeries.at(i, 0) - mean;

        return centered;
    }
}

#endif

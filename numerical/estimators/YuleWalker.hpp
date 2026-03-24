#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "numerical/math/Matrix.hpp"
#include "numerical/math/Toeplitz.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Order>
    class YuleWalker
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "YuleWalker only supports float or QNumber types");
        static_assert(Samples > Order,
            "Number of samples must exceed order");
        static_assert(Order > 0,
            "Order must be positive");

    public:
        using CoefficientsVector = math::Vector<T, Order>;
        using AutocovarianceVector = math::Vector<T, Order + 1>;
        using InputVector = math::Vector<T, Samples>;

        void Fit(const InputVector& x);
        T Predict(const math::Vector<T, Order>& past) const;
        const CoefficientsVector& Coefficients() const;
        T NoiseVariance() const;

    private:
        OPTIMIZE_FOR_SPEED AutocovarianceVector ComputeAutocovariance(const InputVector& x, T mean) const;

        CoefficientsVector coefficients;
        T mean{};
        T noiseVariance{};
    };

    // Implementation //

    template<typename T, std::size_t Samples, std::size_t Order>
    OPTIMIZE_FOR_SPEED
    void YuleWalker<T, Samples, Order>::Fit(const InputVector& x)
    {
        mean = T(0.0f);
        for (std::size_t i = 0; i < Samples; ++i)
            mean += x[i];
        mean = mean / T(static_cast<float>(Samples));

        auto r = ComputeAutocovariance(x, mean);

        if (r[0] == T(0.0f))
        {
            for (std::size_t i = 0; i < Order; ++i)
                coefficients[i] = T(0.0f);
            noiseVariance = T(0.0f);
            return;
        }

        math::Vector<T, Order> autocorrelation;
        for (std::size_t i = 0; i < Order; ++i)
            autocorrelation[i] = r[i];

        math::ToeplitzMatrix<T, Order> toeplitz(autocorrelation);
        auto toeplitzMatrix = toeplitz.ToFullMatrix();

        math::Vector<T, Order> rhs;
        for (std::size_t i = 0; i < Order; ++i)
            rhs[i] = r[i + 1];

        solvers::GaussianElimination<T, Order> solver;
        coefficients = solver.Solve(toeplitzMatrix, rhs);

        noiseVariance = r[0];
        for (std::size_t i = 0; i < Order; ++i)
            noiseVariance -= coefficients[i] * r[i + 1];
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    OPTIMIZE_FOR_SPEED
    T YuleWalker<T, Samples, Order>::Predict(const math::Vector<T, Order>& past) const
    {
        T prediction = mean;
        for (std::size_t i = 0; i < Order; ++i)
            prediction += coefficients[i] * (past[i] - mean);

        return prediction;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    const typename YuleWalker<T, Samples, Order>::CoefficientsVector& YuleWalker<T, Samples, Order>::Coefficients() const
    {
        return coefficients;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    T YuleWalker<T, Samples, Order>::NoiseVariance() const
    {
        return noiseVariance;
    }

    template<typename T, std::size_t Samples, std::size_t Order>
    OPTIMIZE_FOR_SPEED
    typename YuleWalker<T, Samples, Order>::AutocovarianceVector YuleWalker<T, Samples, Order>::ComputeAutocovariance(const InputVector& x, T xMean) const
    {
        AutocovarianceVector r;

        for (std::size_t k = 0; k <= Order; ++k)
        {
            T sum = T(0.0f);
            for (std::size_t t = k; t < Samples; ++t)
                sum += (x[t] - xMean) * (x[t - k] - xMean);

            r[k] = sum / T(static_cast<float>(Samples));
        }

        return r;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class YuleWalker<float, 256, 2>;
    extern template class YuleWalker<float, 128, 4>;
#endif
}

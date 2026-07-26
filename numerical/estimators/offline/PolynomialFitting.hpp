#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <type_traits>

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Degree>
    class PolynomialFitting
    {
        static_assert(std::is_floating_point_v<T>, "PolynomialFitting supports floating-point types");
        static_assert(Samples >= Degree + 1, "Samples must be >= Degree + 1");

    public:
        using CoefficientsVector = math::Matrix<T, Degree + 1, 1>;
        using SamplesVector = math::Matrix<T, Samples, 1>;

        PolynomialFitting() = default;

        OPTIMIZE_FOR_SPEED void Fit(const SamplesVector& x, const SamplesVector& y);
        T Predict(T xVal) const;
        const CoefficientsVector& Coefficients() const;

    private:
        CoefficientsVector coefficients;
    };

    template<typename T, std::size_t Samples, std::size_t Degree>
    OPTIMIZE_FOR_SPEED void PolynomialFitting<T, Samples, Degree>::Fit(const SamplesVector& x, const SamplesVector& y)
    {
        math::Matrix<T, Samples, Degree + 1> v;

        for (std::size_t i = 0; i < Samples; ++i)
        {
            v.at(i, 0) = T{ 1 };
            for (std::size_t j = 1; j <= Degree; ++j)
                v.at(i, j) = v.at(i, j - 1) * x.at(i, 0);
        }

        auto vt = v.Transpose();
        auto normalMatrix = vt * v;
        auto rhs = vt * y;

        coefficients = solvers::SolveSystem<T, Degree + 1, 1>(normalMatrix, rhs);
    }

    template<typename T, std::size_t Samples, std::size_t Degree>
    T PolynomialFitting<T, Samples, Degree>::Predict(T xVal) const
    {
        T acc = coefficients.at(Degree, 0);
        for (std::size_t j = Degree; j > 0; --j)
            acc = acc * xVal + coefficients.at(j - 1, 0);
        return acc;
    }

    template<typename T, std::size_t Samples, std::size_t Degree>
    const typename PolynomialFitting<T, Samples, Degree>::CoefficientsVector&
    PolynomialFitting<T, Samples, Degree>::Coefficients() const
    {
        return coefficients;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PolynomialFitting<float, 8, 2>;
#endif
}

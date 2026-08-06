#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <type_traits>

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Features>
    class TotalLeastSquares
    {
        static_assert(std::is_floating_point_v<T>, "TotalLeastSquares supports floating-point types only");
        static_assert(Samples >= Features + 1, "Samples must be >= Features + 1");

    public:
        using CoefficientsVector = math::Vector<T, Features>;
        using DesignMatrix = math::Matrix<T, Samples, Features>;
        using SamplesVector = math::Vector<T, Samples>;
        using FeaturesVector = math::Vector<T, Features>;

        TotalLeastSquares() = default;

        OPTIMIZE_FOR_SPEED bool Fit(const DesignMatrix& a, const SamplesVector& b);
        T Predict(const FeaturesVector& x) const;
        const CoefficientsVector& Coefficients() const;

    private:
        static constexpr std::size_t Augmented = Features + 1;

        CoefficientsVector coefficients{};
    };

    template<typename T, std::size_t Samples, std::size_t Features>
    OPTIMIZE_FOR_SPEED bool TotalLeastSquares<T, Samples, Features>::Fit(const DesignMatrix& a, const SamplesVector& b)
    {
        math::Matrix<T, Samples, Augmented> m;
        for (std::size_t i = 0; i < Samples; ++i)
        {
            for (std::size_t j = 0; j < Features; ++j)
                m.at(i, j) = a.at(i, j);
            m.at(i, Features) = b.at(i, 0);
        }

        solvers::SingularValueDecomposition<T, Samples, Augmented> svd;
        svd.Decompose(m);

        const auto& v = svd.V();
        T denom = v.at(Features, Features);
        if (math::Abs(denom) < T{ 1e-6f })
            return false;

        for (std::size_t i = 0; i < Features; ++i)
            coefficients.at(i, 0) = -v.at(i, Features) / denom;

        return true;
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    T TotalLeastSquares<T, Samples, Features>::Predict(const FeaturesVector& x) const
    {
        T acc{};
        for (std::size_t i = 0; i < Features; ++i)
            acc += coefficients.at(i, 0) * x.at(i, 0);
        return acc;
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    const typename TotalLeastSquares<T, Samples, Features>::CoefficientsVector&
    TotalLeastSquares<T, Samples, Features>::Coefficients() const
    {
        return coefficients;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class TotalLeastSquares<float, 8, 1>;
    extern template class TotalLeastSquares<float, 10, 2>;
#endif
}

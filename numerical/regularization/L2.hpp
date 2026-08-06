#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/regularization/Regularization.hpp"

namespace regularization
{
    template<typename T, std::size_t Size>
    class L2
        : public Regularization<T, Size>
    {
        static_assert(std::is_floating_point_v<T>, "L2 supports floating-point types");

    public:
        using Vector = typename Regularization<T, Size>::Vector;

        explicit L2(T lambda);
        T Calculate(const Vector& parameters) const override;
        Vector Gradient(const Vector& parameters) const override;

    private:
        T lambda;
    };

    template<typename T, std::size_t Size>
    L2<T, Size>::L2(T lambda)
        : lambda(lambda)
    {}

    template<typename T, std::size_t Size>
    OPTIMIZE_FOR_SPEED T L2<T, Size>::Calculate(const Vector& parameters) const
    {
        T sum{ 0.0f };

        for (std::size_t i = 0; i < Size; ++i)
            sum += parameters[i] * parameters[i];

        return lambda * sum / T(2.0f);
    }

    template<typename T, std::size_t Size>
    OPTIMIZE_FOR_SPEED typename L2<T, Size>::Vector L2<T, Size>::Gradient(const Vector& parameters) const
    {
        Vector gradient;

        for (std::size_t i = 0; i < Size; ++i)
            gradient[i] = lambda * parameters[i];

        return gradient;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class L2<float, 4>;
#endif
}

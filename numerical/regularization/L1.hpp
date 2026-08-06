#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/regularization/Regularization.hpp"

namespace regularization
{
    template<typename T, std::size_t Size>
    class L1
        : public Regularization<T, Size>
    {
        static_assert(std::is_floating_point_v<T>, "L1 supports floating-point types");

    public:
        using Vector = typename Regularization<T, Size>::Vector;

        explicit L1(T lambda);
        T Calculate(const Vector& parameters) const override;
        Vector Gradient(const Vector& parameters) const override;

    private:
        T lambda;
    };

    template<typename T, std::size_t Size>
    L1<T, Size>::L1(T lambda)
        : lambda(lambda)
    {}

    template<typename T, std::size_t Size>
    OPTIMIZE_FOR_SPEED T L1<T, Size>::Calculate(const Vector& parameters) const
    {
        T sum{ 0.0f };

        for (std::size_t i = 0; i < Size; ++i)
            sum += parameters[i] < T(0.0f) ? -parameters[i] : parameters[i];

        return lambda * sum;
    }

    template<typename T, std::size_t Size>
    OPTIMIZE_FOR_SPEED typename L1<T, Size>::Vector L1<T, Size>::Gradient(const Vector& parameters) const
    {
        Vector gradient;

        for (std::size_t i = 0; i < Size; ++i)
        {
            if (parameters[i] > T(0.0f))
                gradient[i] = lambda;
            else if (parameters[i] < T(0.0f))
                gradient[i] = -lambda;
            else
                gradient[i] = T(0.0f);
        }

        return gradient;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class L1<float, 4>;
#endif
}

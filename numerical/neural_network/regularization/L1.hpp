#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "numerical/neural_network/regularization/Regularization.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t Size>
    class L1
        : public Regularization<QNumberType, Size>
    {
    public:
        using Vector = typename Regularization<QNumberType, Size>::Vector;

        explicit L1(QNumberType lambda);
        QNumberType Calculate(const Vector& parameters) const override;
        Vector Gradient(const Vector& parameters) const override;

    private:
        QNumberType lambda;
    };

    // Implementation //

    template<typename QNumberType, std::size_t Size>
    L1<QNumberType, Size>::L1(QNumberType lambda)
        : lambda(lambda)
    {}

    template<typename QNumberType, std::size_t Size>
    OPTIMIZE_FOR_SPEED
    QNumberType L1<QNumberType, Size>::Calculate(const Vector& parameters) const
    {
        QNumberType sum = QNumberType(0.0f);

        for (std::size_t i = 0; i < Size; ++i)
            sum += parameters[i] < QNumberType(0.0f) ? -parameters[i] : parameters[i];

        return lambda * sum;
    }

    template<typename QNumberType, std::size_t Size>
    OPTIMIZE_FOR_SPEED
    typename L1<QNumberType, Size>::Vector L1<QNumberType, Size>::Gradient(const Vector& parameters) const
    {
        Vector gradient;

        for (std::size_t i = 0; i < Size; ++i)
            gradient[i] = parameters[i] > QNumberType(0.0f) ? lambda : (parameters[i] < QNumberType(0.0f) ? -lambda : QNumberType(0.0f));

        return gradient;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class L1<float, 4>;
    extern template class L1<math::Q15, 4>;
    extern template class L1<math::Q31, 4>;
#endif
}

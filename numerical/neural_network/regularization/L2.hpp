#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "numerical/neural_network/regularization/Regularization.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t Size>
    class L2
        : public Regularization<QNumberType, Size>
    {
    public:
        using Vector = typename Regularization<QNumberType, Size>::Vector;

        explicit L2(QNumberType lambda);
        QNumberType Calculate(const Vector& parameters) const override;
        Vector Gradient(const Vector& parameters) const override;

    private:
        QNumberType lambda;
    };

    // Implementation //

    template<typename QNumberType, std::size_t Size>
    L2<QNumberType, Size>::L2(QNumberType lambda)
        : lambda(lambda)
    {}

    template<typename QNumberType, std::size_t Size>
    OPTIMIZE_FOR_SPEED
    QNumberType L2<QNumberType, Size>::Calculate(const Vector& parameters) const
    {
        QNumberType sum = QNumberType(0.0f);

        for (std::size_t i = 0; i < Size; ++i)
            sum += parameters[i] * parameters[i];

        return QNumberType(math::ToFloat(lambda * sum) / 2.0f);
    }

    template<typename QNumberType, std::size_t Size>
    OPTIMIZE_FOR_SPEED
    typename L2<QNumberType, Size>::Vector L2<QNumberType, Size>::Gradient(const Vector& parameters) const
    {
        Vector gradient;

        for (std::size_t i = 0; i < Size; ++i)
            gradient[i] = lambda * parameters[i];

        return gradient;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class L2<float, 4>;
    extern template class L2<math::Q31, 4>;
#endif
}

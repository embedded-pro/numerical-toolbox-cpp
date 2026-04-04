#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/neural_network/losses/Loss.hpp"
#include "numerical/regularization/Regularization.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class MeanSquaredError
        : public Loss<QNumberType, NumberOfFeatures>
    {
    public:
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

        MeanSquaredError(const Vector& target, regularization::Regularization<QNumberType, NumberOfFeatures>& regularization);

        QNumberType Cost(const Vector& parameters) override;
        Vector Gradient(const Vector& parameters) override;

    private:
        Vector target;
        regularization::Regularization<QNumberType, NumberOfFeatures>& regularization;
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    MeanSquaredError<QNumberType, NumberOfFeatures>::MeanSquaredError(const Vector& target, regularization::Regularization<QNumberType, NumberOfFeatures>& regularization)
        : target(target)
        , regularization(regularization)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    OPTIMIZE_FOR_SPEED
        QNumberType
        MeanSquaredError<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters)
    {
        QNumberType cost = QNumberType(0.0f);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            QNumberType diff = parameters[i] - target[i];
            cost += diff * diff;
        }

        return QNumberType(math::ToFloat(cost) / 2.0f) + regularization.Calculate(parameters);
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    OPTIMIZE_FOR_SPEED
        typename MeanSquaredError<QNumberType, NumberOfFeatures>::Vector
        MeanSquaredError<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters)
    {
        Vector gradient;
        auto regGradient = regularization.Gradient(parameters);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
            gradient[i] = parameters[i] - target[i] + regGradient[i];

        return gradient;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class MeanSquaredError<float, 2>;
    extern template class MeanSquaredError<math::Q15, 2>;
    extern template class MeanSquaredError<math::Q31, 2>;
#endif
}

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/neural_network/losses/Loss.hpp"
#include "numerical/neural_network/regularization/Regularization.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class MeanAbsoluteError
        : public Loss<QNumberType, NumberOfFeatures>
    {
    public:
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

        MeanAbsoluteError(const Vector& target, Regularization<QNumberType, NumberOfFeatures>& regularization);
        QNumberType Cost(const Vector& parameters) override;
        Vector Gradient(const Vector& parameters) override;

    private:
        Vector target;
        Regularization<QNumberType, NumberOfFeatures>& regularization;
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    MeanAbsoluteError<QNumberType, NumberOfFeatures>::MeanAbsoluteError(const Vector& target, Regularization<QNumberType, NumberOfFeatures>& regularization)
        : target(target)
        , regularization(regularization)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    OPTIMIZE_FOR_SPEED
        QNumberType
        MeanAbsoluteError<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters)
    {
        QNumberType cost = QNumberType(0.0f);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            auto diff = parameters[i] - target[i];
            cost += diff < QNumberType(0.0f) ? -diff : diff;
        }

        return cost + regularization.Calculate(parameters);
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    OPTIMIZE_FOR_SPEED
        typename MeanAbsoluteError<QNumberType, NumberOfFeatures>::Vector
        MeanAbsoluteError<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters)
    {
        Vector gradient;
        auto regGradient = regularization.Gradient(parameters);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            auto diff = parameters[i] - target[i];
            gradient[i] = (diff > QNumberType(0.0f) ? QNumberType(0.9999f) : QNumberType(-0.9999f)) + regGradient[i];
        }

        return gradient;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class MeanAbsoluteError<float, 2>;
    extern template class MeanAbsoluteError<math::Q15, 2>;
    extern template class MeanAbsoluteError<math::Q31, 2>;
#endif
}

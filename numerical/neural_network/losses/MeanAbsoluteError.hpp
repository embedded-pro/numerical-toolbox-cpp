#ifndef NEURAL_NETWORK_LOSSES_MEAN_ABSOLUTE_ERROR_HPP
#define NEURAL_NETWORK_LOSSES_MEAN_ABSOLUTE_ERROR_HPP

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

        MeanAbsoluteError(const Vector& target, const Regularization<QNumberType, NumberOfFeatures>& regularization);
        QNumberType Cost(const Vector& parameters) override;
        Vector Gradient(const Vector& parameters) override;

    private:
        Vector target;
        Regularization<QNumberType, NumberOfFeatures>& regularization;
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    MeanAbsoluteError<QNumberType, NumberOfFeatures>::MeanAbsoluteError(const Vector& target, const Regularization<QNumberType, NumberOfFeatures>& regularization)
        : target(target)
        , regularization(regularization)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    QNumberType MeanAbsoluteError<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters)
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
    typename MeanAbsoluteError<QNumberType, NumberOfFeatures>::Vector MeanAbsoluteError<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters)
    {
        Vector gradient;

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            auto diff = parameters[i] - target[i];
            gradient[i] = diff > QNumberType(0.0f) ? QNumberType(0.9999f) : QNumberType(-0.9999f);
        }

        return gradient + regularization.Calculate(parameters);
    }
}

#endif

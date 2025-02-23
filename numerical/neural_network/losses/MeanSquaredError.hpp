#ifndef NEURAL_NETWORK_MEAN_SQUARED_ERROR_HPP
#define NEURAL_NETWORK_MEAN_SQUARED_ERROR_HPP

#include "neural_network/Regularization.hpp"
#include "optimizers/Loss.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class MeanSquaredError
        : public optimizer::Loss<QNumberType, NumberOfFeatures>
    {
    public:
        using Vector = typename optimizer::Loss<QNumberType, NumberOfFeatures>::Vector;

        MeanSquaredError(const Vector& target, const Regularization<QNumberType, NumberOfFeatures>& regularization);

        QNumberType Cost(const Vector& parameters, QNumberType regularizationScale) override;
        Vector Gradient(const Vector& parameters, QNumberType regularizationScale) override;

    private:
        Vector target;
        Regularization<QNumberType, NumberOfFeatures>& regularization;
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    MeanSquaredError<QNumberType, NumberOfFeatures>::MeanSquaredError(const Vector& target, const Regularization<QNumberType, NumberOfFeatures>& regularization)
        : target(target)
        , regularization(regularization)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    QNumberType MeanSquaredError<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters, QNumberType regularizationScale)
    {
        QNumberType cost = QNumberType(0);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            QNumberType diff = parameters[i] - target[i];
            cost += diff * diff;
        }

        cost /= QNumberType(2);

        return cost + regularizationScale * regularization.Calculate(parameters);
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    typename MeanSquaredError<QNumberType, NumberOfFeatures>::Vector MeanSquaredError<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters, QNumberType regularizationScale)
    {
        Vector gradient;

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
            gradient[i] = parameters[i] - target[i];

        return gradient + regularization.Calculate(parameters) * regularizationScale;
    }
}

#endif

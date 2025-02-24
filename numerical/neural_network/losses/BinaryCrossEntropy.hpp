#ifndef NEURAL_NETWORK_LOSSES_BINARY_CROSS_ENTROPY_HPP
#define NEURAL_NETWORK_LOSSES_BINARY_CROSS_ENTROPY_HPP

#include "numerical/neural_network/losses/Loss.hpp"
#include "numerical/neural_network/regularization/Regularization.hpp"
#include <cmath>

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class BinaryCrossEntropy
        : public Loss<QNumberType, NumberOfFeatures>
    {
    public:
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

        BinaryCrossEntropy(const Vector& target, Regularization<QNumberType, NumberOfFeatures>& regularization);
        QNumberType Cost(const Vector& parameters) override;
        Vector Gradient(const Vector& parameters) override;

    private:
        Vector target;
        Regularization<QNumberType, NumberOfFeatures>& regularization;
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    BinaryCrossEntropy<QNumberType, NumberOfFeatures>::BinaryCrossEntropy(
        const Vector& target,
        Regularization<QNumberType, NumberOfFeatures>& regularization)
        : target(target)
        , regularization(regularization)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    QNumberType BinaryCrossEntropy<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters)
    {
        QNumberType cost = QNumberType(0.0f);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            QNumberType pred = std::max(std::min(parameters[i], QNumberType(0.9999f)), QNumberType(0.0001f));

            cost += -(target[i] * std::log(math::ToFloat(pred)) +
                      (QNumberType(0.9999f) - target[i]) * std::log(math::ToFloat(QNumberType(0.9999f) - pred)));
        }

        return cost + regularization.Calculate(parameters);
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    typename BinaryCrossEntropy<QNumberType, NumberOfFeatures>::Vector
    BinaryCrossEntropy<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters)
    {
        Vector gradient;

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            QNumberType pred = std::max(std::min(parameters[i], QNumberType(0.9999f)), QNumberType(0.0001f));

            gradient[i] = (pred - target[i]) / (pred * (QNumberType(0.9999f) - pred)) + regularization.Calculate(parameters);
        }

        return gradient;
    }
}

#endif

#ifndef NEURAL_NETWORK_LOSSES_CATEGORICAL_CROSS_ENTROPY_HPP
#define NEURAL_NETWORK_LOSSES_CATEGORICAL_CROSS_ENTROPY_HPP

#include "numerical/neural_network/activation/Softmax.hpp"
#include "numerical/neural_network/losses/Loss.hpp"
#include "numerical/neural_network/regularization/Regularization.hpp"
#include <cmath>

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class CategoricalCrossEntropy
        : public Loss<QNumberType, NumberOfFeatures>
    {
    public:
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

        CategoricalCrossEntropy(const Vector& target, const Regularization<QNumberType, NumberOfFeatures>& regularization);
        QNumberType Cost(const Vector& parameters) override;
        Vector Gradient(const Vector& parameters) override;

    private:
        Vector target;
        Regularization<QNumberType, NumberOfFeatures>& regularization;
        Softmax<QNumberType> softmax;

        Vector ComputeSoftmaxProbabilities(const Vector& x) const;
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    CategoricalCrossEntropy<QNumberType, NumberOfFeatures>::CategoricalCrossEntropy(
        const Vector& target,
        const Regularization<QNumberType, NumberOfFeatures>& regularization)
        : target(target)
        , regularization(regularization)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    typename CategoricalCrossEntropy<QNumberType, NumberOfFeatures>::Vector
    CategoricalCrossEntropy<QNumberType, NumberOfFeatures>::ComputeSoftmaxProbabilities(const Vector& x) const
    {
        Vector output;
        QNumberType sum = QNumberType(0);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            output[i] = softmax.Forward(x[i]);
            sum += output[i];
        }

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            output[i] = output[i] / sum;
            output[i] = std::max(std::min(output[i], QNumberType(0.9999f)), QNumberType(0.0001f));
        }

        return output;
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    QNumberType CategoricalCrossEntropy<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters)
    {
        Vector probabilities = ComputeSoftmaxProbabilities(parameters);
        QNumberType cost = QNumberType(0);

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
            cost += -target[i] * std::log(math::ToFloat(probabilities[i]));

        return cost + regularization.Calculate(parameters);
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    typename CategoricalCrossEntropy<QNumberType, NumberOfFeatures>::Vector
    CategoricalCrossEntropy<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters)
    {
        Vector probabilities = ComputeSoftmaxProbabilities(parameters);
        Vector gradient;

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
            gradient[i] = probabilities[i] - target[i];

        return gradient + regularization.Calculate(parameters);
    }
}

#endif

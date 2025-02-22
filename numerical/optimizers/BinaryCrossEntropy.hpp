#ifndef OPTIMIZERS_BINARY_CROSS_ENTROPY_HPP
#define OPTIMIZERS_BINARY_CROSS_ENTROPY_HPP

#include "Loss.hpp"
#include <cmath>

namespace optimizer
{

    template<typename QNumberType, std::size_t NumberOfFeatures>
    class BinaryCrossEntropyLoss : public Loss<QNumberType, NumberOfFeatures>
    {
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

    public:
        BinaryCrossEntropyLoss(const Vector& target, const Vector& features)
            : target_(target)
            , features_(features)
        {}

        QNumberType Cost(const Vector& parameters, QNumberType regularization) override
        {
            Vector predictions = ComputePredictions(parameters);
            QNumberType pred = std::max(std::min(predictions[0], static_cast<QNumberType>(1.0 - epsilon_)), epsilon_);
            QNumberType cross_entropy = -target_[0] * std::log(pred) - (static_cast<QNumberType>(1) - target_[0]) * std::log(static_cast<QNumberType>(1) - pred);

            if (regularization > 0)
            {
                QNumberType l2_term = 0;
                for (std::size_t i = 0; i < NumberOfFeatures; ++i)
                    l2_term += parameters[i] * parameters[i];

                cross_entropy += (regularization * l2_term) / 2;
            }

            return cross_entropy;
        }

        Vector Gradient(const Vector& parameters, QNumberType regularization) override
        {
            Vector gradient;

            Vector predictions = ComputePredictions(parameters);
            QNumberType error = predictions[0] - target_[0];

            for (std::size_t i = 0; i < NumberOfFeatures; ++i)
            {
                gradient[i] = error * features_[i];

                if (regularization > 0)
                    gradient[i] += regularization * parameters[i];
            }

            return gradient;
        }

    private:
        const Vector& target_;
        const Vector& features_;
        const QNumberType epsilon_ = static_cast<QNumberType>(1e-7);

        QNumberType Sigmoid(QNumberType x) const
        {
            return static_cast<QNumberType>(1) / (static_cast<QNumberType>(1) + std::exp(-x));
        }

        Vector ComputePredictions(const Vector& parameters) const
        {
            Vector predictions;
            QNumberType logit = 0;

            for (std::size_t i = 0; i < NumberOfFeatures; ++i)
                logit += parameters[i] * features_[i];

            predictions[0] = Sigmoid(logit);

            return predictions;
        }
    };
}

#endif

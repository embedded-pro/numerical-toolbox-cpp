#ifndef LOSSES_CROSS_ENTROPY_HPP
#define LOSSES_CROSS_ENTROPY_HPP

#include "Loss.hpp"

namespace optimizer
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class CrossEntropyLoss
        : public Loss<QNumberType, NumberOfFeatures>
    {
    public:
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

        CrossEntropyLoss(const Vector& target, const Vector& prediction);

        QNumberType Cost(const Vector& parameters, QNumberType regularization) override;
        Vector Gradient(const Vector& parameters, QNumberType regularization) override;

    private:
        const Vector& target;
        const Vector& prediction;
        const QNumberType epsilon = static_cast<QNumberType>(1e-7);
    };

    // Implementation //

    template<typename QNumberType, std::size_t NumberOfFeatures>
    CrossEntropyLoss<QNumberType, NumberOfFeatures>::CrossEntropyLoss(const Vector& target, const Vector& prediction)
        : target(target)
        , prediction(prediction)
    {}

    template<typename QNumberType, std::size_t NumberOfFeatures>
    QNumberType CrossEntropyLoss<QNumberType, NumberOfFeatures>::Cost(const Vector& parameters, QNumberType regularization)
    {
        QNumberType cross_entropy = 0;

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            QNumberType pred = std::max(std::min(prediction[i], static_cast<QNumberType>(1.0 - epsilon)), epsilon);
            cross_entropy -= target[i] * std::log(pred);
        }

        QNumberType l2_term = 0;
        if (regularization > 0)
        {
            for (std::size_t i = 0; i < NumberOfFeatures; ++i)
                l2_term += parameters[i] * parameters[i];

            l2_term *= (regularization / 2);
        }

        return cross_entropy + l2_term;
    }

    template<typename QNumberType, std::size_t NumberOfFeatures>
    typename CrossEntropyLoss<QNumberType, NumberOfFeatures>::Vector CrossEntropyLoss<QNumberType, NumberOfFeatures>::Gradient(const Vector& parameters, QNumberType regularization)
    {
        Vector gradient;

        for (std::size_t i = 0; i < NumberOfFeatures; ++i)
        {
            QNumberType pred = std::max(std::min(prediction[i], static_cast<QNumberType>(1.0 - epsilon)), epsilon);
            gradient[i] = -target[i] / pred;
        }

        if (regularization > 0)
            for (std::size_t i = 0; i < NumberOfFeatures; ++i)
                gradient[i] += regularization * parameters[i];

        return gradient;
    }
}

#endif

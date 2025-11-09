#ifndef NEURAL_NETWORK_L1_HPP
#define NEURAL_NETWORK_L1_HPP

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

    private:
        QNumberType lambda;
    };

    // Implementation //

    template<typename QNumberType, std::size_t Size>
    L1<QNumberType, Size>::L1(QNumberType lambda)
        : lambda(lambda)
    {}

    template<typename QNumberType, std::size_t Size>
    QNumberType L1<QNumberType, Size>::Calculate(const Vector& parameters) const
    {
        QNumberType sum = QNumberType(0.0f);

        for (std::size_t i = 0; i < Size; ++i)
            sum += parameters[i] < QNumberType(0.0f) ? -parameters[i] : parameters[i];

        return lambda * sum;
    }
}

#endif

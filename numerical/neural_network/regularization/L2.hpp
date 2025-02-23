#ifndef NEURAL_NETWORK_L2_HPP
#define NEURAL_NETWORK_L2_HPP

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

    private:
        QNumberType lambda;
    };

    // Implementation //

    template<typename QNumberType, std::size_t Size>
    L2<QNumberType, Size>::L2(QNumberType lambda)
        : lambda(lambda)
    {}

    template<typename QNumberType, std::size_t Size>
    QNumberType L2<QNumberType, Size>::Calculate(const Vector& parameters) const
    {
        QNumberType sum = QNumberType(0.0f);

        for (std::size_t i = 0; i < Size; ++i)
            sum += parameters[i] * parameters[i];

        return QNumberType(math::ToFloat(lambda * sum) / 2.0f);
    }
}

#endif

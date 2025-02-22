#ifndef OPTIMIZERS_LOSS_HPP
#define OPTIMIZERS_LOSS_HPP

#include "math/Matrix.hpp"

namespace optimizer
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class Loss
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "Loss can only be instantiated with math::QNumber types.");

    public:
        using Vector = math::Vector<QNumberType, NumberOfFeatures>;

        virtual QNumberType Cost(const Vector& parameters, QNumberType regularization) = 0;
        virtual Vector Gradient(const Vector& parameters, QNumberType regularization) = 0;
    };
}

#endif
